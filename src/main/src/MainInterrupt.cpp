#include "../../../src-vivado/output/motors/include/Motors.hpp"
#include "../../../src-vivado/sensors/ahrs/include/AHRS.hpp"
#include "../../../src-vivado/sensors/imu/include/IMU.hpp"
#include "../../../src-vivado/sensors/rc/include/RC.hpp"
#include "../../../src-vivado/sensors/sonar/include/Sonar.hpp"
#include <BaremetalCommunicationDef.hpp>
#include <ControllerInstances.hpp>
#include <MainInterrupt.hpp>
#include <MiscInstances.hpp>
#include <SharedMemoryInstances.hpp>
#include <TiltCorrection.hpp>

real_t calculateYawJump(float yaw) {

    /* Whenever the yaw passes 10 degrees (0.1745 rad), it will jump to -10
       degrees and vice versa. */
    static constexpr real_t MAX_YAW_RADS = 0.1745;

    /* The size of the interval is 2*MAX_YAW. */
    real_t size = 2 * MAX_YAW_RADS;

    /* Calculate yaw value in [0, 2*MAX_YAW]. */
    real_t modYaw = fmod(fmod(yaw, size) + size, size);

    /* Calculate yaw value in [-MAX_YAW, +MAX_YAW]. */
    modYaw -= MAX_YAW_RADS;

    /* Return the yaw jump. */
    return modYaw - yaw;
}

void updateLEDs(bool isInterruptRunning, bool isControllerArmed,
                bool isAutonomous, bool isWPTActive) {

    int ledOutput = 0x0;

    /* LED 0 is lit when the interrupts are running too slowly. */
    if (isInterruptRunning)
        ledOutput += 0x1;

    /* LED 1 is lit when the controller is armed. */
    if (isControllerArmed)
        ledOutput += 0x2;

    /* LED 2 is lit when the drone is in autonomous mode. */
    if (isAutonomous)
        ledOutput += 0x4;

    /* LED 3 is lit when wireless power transfer is active. */
    if (isWPTActive)
        ledOutput += 0x8;

    /* Write value to LEDs. */
    writeValueToLEDs(ledOutput);
}

// Called by src-vivado every 238 Hz after initialization/calibration is complete.
void updateMainFSM() {

    /* Previous flight mode initialized when the function is first called. */
    static FlightMode previousFlightMode = FlightMode::UNINITIALIZED;

    /* Keep the attitude controller's state estimate near the unit quaternion
       [1;0;0;0] to ensure the stability of the control system. Whenever the yaw
       passes 10 degrees (0.1745 rad), it will jump to -10 degrees and vice
       versa. */
    real_t yawJump =
        calculateYawJump(attitudeController.getOrientationEuler().yaw);
    attitudeController.calculateJumpedQuaternions(yawJump);

    /* Read RC data and update the global struct. */
    setRCInput(readRC());

    /* Read IMU measurement and update the AHRS. */
    updateAHRS(readIMU());
    Quaternion jumpedOrientationMeasurement = getJumpedOrientation(yawJump);

    /* Read sonar measurement and correct it using the drone's orientation. */
    bool hasNewSonarMeasurement = readSonar();
    real_t sonarMeasurement;
    real_t correctedSonarMeasurement;
    if (hasNewSonarMeasurement) {
        sonarMeasurement          = getFilteredSonarMeasurement();
        correctedSonarMeasurement = getCorrectedHeight(
            sonarMeasurement, attitudeController.getOrientationEstimate());
    }

    /* Read IMP measurement from shared memory and correct it using the sonar
       measurement and the drone's orientation. */
    bool hasNewIMPMeasurement = false;
    real_t yawMeasurement;
    Position correctedPositionMeasurement;
    if (visionComm->isDoneWriting()) {
        hasNewIMPMeasurement         = true;
        VisionData visionData        = visionComm->read();
        impPositionMeasurement       = visionData.position;
        impYawMeasurement            = visionData.yawAngle;
        correctedPositionMeasurement = getCorrectedPosition(
            ColVector<2>{VisionData.position.x, VisionData.position.y},
            sonarMeasurement, attitudeController.getOrientationEstimate());
    }

    /* Implement main FSM logic. The transformed motor signals will be
       calculated based on the current flight mode, the measurements and the
       current state of the controllers. */
    AttitudeControlSignal uxyz;
    real_t uc;
    switch (rcManager.getFlightMode()) {
        case FlightMode::MANUAL:
            // TODO: arming check
            // TODO: gradual thrust change if last mode was altitude-hold
            /* Common thrust comes directly from the RC. */
            uc = rcManager.getThrottle();

            /* Update the attitude controller's reference using the RC. */
            attitudeController.updateRCReference();

            /* Calculate the torque motor signals. */
            uxyz = attitudeController.updateControlSignal(uc);
            break;
        case FlightMode::ALTITUDE_HOLD:
            if (previousFlightMode == FlightMode::MANUAL)
                altitudeController.init();

            if (hasNewSonarMeasurement) {

                /* Update the altitude controller's reference using the RC. */
                altitudeController.updateRCReference();

                /* Common thrust is calculated by the altitude controller. */
                uc = inputBias.getThrustBias() +
                     altitudeController.updateControlSignal().ut;

                /* Calculate the torque motor signals. */
                uxyz = attitudeController.updateControlSignal(uc);
            }
            break;
        case FlightMode::AUTONOMOUS:
            // TODO: when should we initGround or initAir?
            // TODO: should we tell IMP to reset their measurement?
            if (hasNewIMPMeasurement) {

                /* Autonomous controller, using position of the drone. */
                AutonomousOutput output =
                    autonomousController.update(correctedPositionMeasurement);

                /* Position controller, using AutonomousOutput. */
                PositionControlSignal q12ref;
                if (output.updatePositionController) {

                    /* Calculate current position state estimate. */
                    if (output.trustAccelerometerForPosition)
                        positionController.updateObserverBlind();
                    ;
                    else positionController.updateObserver(
                        attitudeController.getOrientationEstimate(), getTime(),
                        {correctedPositionMeasurement});

                    /* Calculate control signal. */
                    q12ref = positionController.updateControlSignal(
                        {output.referencePosition});
                } else {
                    q12ref = {0.0, 0.0};
                }

                /* Altitude controller, using AutonomousOutput. */
                if (output.bypassAltitudeController) {
                    uc = output.commonThrust;
                } else {
                    altitudeController.setReference({output.referenceHeight});
                    uc = inputBias.getThrustBias() +
                         altitudeController.updateControlSignal().ut;
                }

                /* Calculate the torque motor signals. */
                real_t q1        = q12ref.q1ref;
                real_t q2        = q12ref.q2ref;
                real_t q0        = 1 - sqrt(q1 * q1 + q2 * q2);
                real_t pitch     = inputBias.getPitchBias();
                real_t roll      = inputBias.getRollBias();
                Quaternion quat1 = Quaternion(q0, q1, q2, 0);
                Quaternion quat2 = EulerAngles::eul2quat(0.0, pitch, roll);
                attitudeController.setReference(quat1 + quat2);
                attitudeController.setReferenceEuler({quat1 + quat2});
                uxyz = attitudeController.updateControlSignal(uc);
            }
            break;
        case FlightMode::UNINITIALIZED:
            /* We will never get here because readRC() cannot return an
            uninitialized flight mode. */
            break;
    }

    /* Transform the motor signals and output to the motors. */
    MotorDutyCycles dutyCycles = transformAttitudeControlSignal(uxyz, uc);
    outputMotorPWM(dutyCycles.v0, dutyCycles.v1, dutyCycles.v2, dutyCycles.v2);

    // TODO: kill if the drone tilts too far? (droneControllersActivated flag)
    // TODO: gradual thrust change
    // TODO: update buzzer
    // TODO: escs startup script
    // TODO: update configuration

    /* Update input biases. */
    inputBias.updatePitchBias(attitudeController.getReferenceEuler().pitch,
                              rcManager.getFlightMode());
    inputBias.updateRollBias(attitudeController.getReferenceEuler().roll,
                             rcManager.getFlightMode());
    inputBias.updateThrustBias(uc);

    /* Update the Kalman Filters (the position controller doesn't use one). */
    attitudeController.updateObserver({orientationMeasurement}, yawJump);
    altitudeController.updateObserver({correctedSonarMeasurement});

    /* Store flight mode. */
    previousFlightMode = rcManager.getFlightMode();
}

void update() {

    /* Test pin high to probe length of interrupt. */
    writeValueToTestPin(true);

    /* Update the drone's clock. */
    incrementTickCount();

    /* Whether an interrupt is currently running. */
    static bool isInterruptRunning = false;
    /* IMU bias should be calculated before use. */
    static bool isIMUCalibrated = false;
    /* AHRS should calibrate with accelerometer before use. */
    static bool isAHRSInitialized = false;

    /* Update LEDs. */
    updateLEDs(isInterruptRunning, armedManager.isArmed(),
               rcManager.getFlightMode() == FlightMode::AUTONOMOUS,
               rcManager.getWPTMode() == WPTMode::ON);

    /* Phase 1: Calibrate IMU. */
    if (!isIMUCalibrated) {
        isIMUCalibrated = calibrateIMU();
    }
    /* Phase 2: Initialize AHRS. */
    else if (!isAHRSInitialized) {
        initAHRS();
        isAHRSInitialized = true;
    }
    /* Phase 3: Main operation. */
    else {
        updateMainFSM();
    }

    /* Output to LEDS. */
    updateLEDs()

        /* Test pin low to probe length of interrupt. */
        writeValueToTestPin(false);
}
