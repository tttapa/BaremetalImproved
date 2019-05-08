#include <AHRS.hpp>
#include <AxiGpio.hpp>
#include <BaremetalCommunicationDef.hpp>
#include <ControllerInstances.hpp>
#include <IMU.hpp>
#include <MainInterrupt.hpp>
#include <MiscInstances.hpp>
#include <Motors.hpp>
#include <OutputValues.hpp>
#include <RC.hpp>
#include <RCValues.hpp>
#include <SensorValues.hpp>
#include <SharedMemoryInstances.hpp>
#include <Sonar.hpp>
#include <TiltCorrection.hpp>

/* Whether an interrupt is currently running. */
static bool isInterruptRunning = false;

void mainLoop() { isInterruptRunning = false; }

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

    // TODO: cleanup

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
    setYawJump(calculateYawJump(attitudeController.getOrientationEuler().yaw));
    attitudeController.calculateJumpedQuaternions(getYawJump());

    /* Read RC data and update the global struct. */
    setRCInput(readRC());

    /* Read IMU measurement and update the AHRS. */
    setIMUMeasurement(readIMU());
    setAHRSQuat(updateAHRS(getIMUMeasurement()));
    setJumpedAHRSQuat(getJumpedOrientation(getYawJump()));

    /* Read sonar measurement and correct it using the drone's orientation. */
    bool hasNewSonarMeasurement = readSonar();
    if (hasNewSonarMeasurement) {
        setSonarMeasurement(getFilteredSonarMeasurement());
        setCorrectedSonarMeasurement(getCorrectedHeight(
            getSonarMeasurement(), attitudeController.getOrientationQuat()));
    }

    /* Read IMP measurement from shared memory and correct it using the sonar
       measurement and the drone's orientation. */
    bool hasNewIMPMeasurement = false;
    if (visionComm->isDoneWriting()) {
        hasNewIMPMeasurement  = true;
        VisionData visionData = visionComm->read();
        setPositionMeasurement(visionData.position);
        setYawMeasurement(visionData.yawAngle);
        ColVector<2> correctedPosition = getCorrectedPosition(
            ColVector<2>{visionData.position.x, visionData.position.y},
            getSonarMeasurement(), attitudeController.getOrientationQuat());
        setCorrectedPositionMeasurement(
            {correctedPosition[0],
             correctedPosition[1]});  // TODO: adapter function
    }

    /* Save the measurements for the logger. */

    /* Implement main FSM logic. The transformed motor signals will be
       calculated based on the current flight mode, the measurements and the
       current state of the controllers. */
    AttitudeControlSignal uxyz;  // TODO: variable names
    real_t uc = 0.0;
    /// Remember last cycle's common thrust for GTC.
    static real_t ucLast = 0.0;
    switch (getFlightMode()) {
        case FlightMode::MANUAL:

            /* Check whether the drone should be armed or disarmed. This should
               only occur in manual mode. */
            armedManager.update();

            /* Start gradual thrust change if last mode was altitude hold. */
            gtcManager.start(ucLast);

            /* Common thrust comes directly from the RC. */
            uc = getThrottle();

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
            if (previousFlightMode == FlightMode::ALTITUDE_HOLD)
                autonomousController.initAir(Position{0.5, 0.5},
                                             {getCorrectedSonarMeasurement()});
            // TODO: should we tell IMP to reset their measurement?
            if (hasNewIMPMeasurement) {
                // TODO: in autonomous mode, update AHRS's yaw

                /* Autonomous controller, using position of the drone. */
                AutonomousOutput output = autonomousController.update(
                    getCorrectedPositionMeasurement());

                /* Position controller, using AutonomousOutput. */
                PositionControlSignal q12ref;
                if (output.updatePositionController) {

                    /* Calculate current position state estimate. */
                    if (output.trustAccelerometerForPosition)
                        positionController.updateObserverBlind(
                            attitudeController.getOrientationQuat());

                    else
                        positionController.updateObserver(
                            attitudeController.getOrientationQuat(), getTime(),
                            {getCorrectedPositionMeasurement().x,
                             getCorrectedPositionMeasurement().y});
                    // TODO: adapter

                    /* Calculate control signal. */
                    q12ref = positionController.updateControlSignal(
                        {output.referencePosition.x,
                         output.referencePosition.y});
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
                real_t yaw       = 0.0;
                real_t pitch     = inputBias.getPitchBias();
                real_t roll      = inputBias.getRollBias();
                Quaternion quat1 = Quaternion(q0, q1, q2, 0);
                Quaternion quat2 = EulerAngles::eul2quat({yaw, pitch, roll});
                attitudeController.setReferenceEuler({quat1 + quat2});
                uxyz = attitudeController.updateControlSignal(uc);
            }
            break;
        case FlightMode::UNINITIALIZED:
            /* We will never get here because readRC() cannot return an
            uninitialized flight mode. */
            break;
    }

    /* Remember common thrust for next clock cycle, which is needed to start the
       Gradual Thrust Change (GTC). */
    ucLast = uc;

    /* Pass the common thrust through the ESC startup script if it's enabled. */
    if (escStartupScript.isEnabled())
        uc = escStartupScript.update(uc);

    /* Gradual thrust change active? */
    if (gtcManager.isBusy()) {
        gtcManager.update();
        uc = gtcManager.getThrust();
    }

    /* Drone calibration? */
    if (configManager.getControllerConfiguration() == CALIBRATION_MODE) {
        if (getThrottle() >= 0.50)
            uc = 1.0;
        else
            uc = 0.0;
    }

    /* Save the common thrust for the logger. */
    setCommonThrust(uc);

    /* Transform the motor signals and output to the motors. */
    MotorDutyCycles dutyCycles = transformAttitudeControlSignal(uxyz, uc);
    setDutyCycles(dutyCycles);
    if (armedManager.isArmed())
        outputMotorPWM(dutyCycles.v0, dutyCycles.v1, dutyCycles.v2,
                       dutyCycles.v2);

    /* Update the controller configuration if the common thrust is near zero. */
    configManager.update(uc);

    /* Update the buzzer. */
    buzzerManager.update();

    // TODO: kill if the drone tilts too far? (droneControllersActivated flag)

    /* Update input biases. */
    // TODO: remember input bias so we can fly immediately in autonomous
    inputBias.updatePitchBias(attitudeController.getReferenceEuler().pitch,
                              getFlightMode());
    inputBias.updateRollBias(attitudeController.getReferenceEuler().roll,
                             getFlightMode());
    inputBias.updateThrustBias(uc);

    /* Update the Kalman Filters (the position controller doesn't use one). */
    // TODO: make sure this is correct... attitude should update its euler
    //       representation with getJumpedOrientation() - previousOrientation
    attitudeController.updateObserver(
        {
            getJumpedAHRSQuat(),
            getGyroMeasurement().gx,
            getGyroMeasurement().gy,
            getGyroMeasurement().gz,
        },
        getYawJump());
    altitudeController.updateObserver({getCorrectedSonarMeasurement()});

    /* Store flight mode. */
    previousFlightMode = getFlightMode();
}

void update() {

    /* Test pin high to probe length of interrupt. */
    writeValueToTestPin(true);

    /* Update the drone's clock. */
    incrementTickCount();

    /* IMU bias should be calculated before use. */
    static bool isIMUCalibrated = false;
    /* AHRS should calibrate with accelerometer before use. */
    static bool isAHRSInitialized = false;

    /* Update LEDs. */
    updateLEDs(isInterruptRunning, armedManager.isArmed(),
               getFlightMode() == FlightMode::AUTONOMOUS,
               getWPTMode() == WPTMode::ON);

    /* Phase 1: Calibrate IMU. */
    if (!isIMUCalibrated) {
        isIMUCalibrated = calibrateIMUStep();
    }
    /* Phase 2: Initialize AHRS. */
    else if (!isAHRSInitialized) {
        initAHRS(readIMU());
        isAHRSInitialized = true;
    }
    /* Phase 3: Main operation. */
    else {
        updateMainFSM();
    }

    /* Test pin low to probe length of interrupt. */
    writeValueToTestPin(false);
}
