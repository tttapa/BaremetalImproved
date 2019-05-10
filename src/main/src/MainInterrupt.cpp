#include <MainInterrupt.hpp>
#include <iostream>

/* Includes from src. */
#include <BaremetalCommunicationDef.hpp>
#include <ControllerInstances.hpp>
#include <LoggerValues.hpp>
#include <MiscInstances.hpp>
#include <RCValues.hpp>
#include <SharedMemoryInstances.hpp>
#include <TiltCorrection.hpp>
#include <Time.hpp>
#include <logger.drone.hpp>

/* Includes from src-vivado. */
#include <output/Motors.hpp>
#include <platform/AxiGpio.hpp>
#include <sensors/AHRS.hpp>
#include <sensors/IMU.hpp>
#include <sensors/RC.hpp>
#include <sensors/Sonar.hpp>

/** Whether an interrupt is currently running. */
volatile bool isInterruptRunning = false;

real_t calculateYawJump(float yaw) {

    /* Whenever the yaw passes 10 degrees (0.1745 rad), it will jump to -10
       degrees and vice versa. */
    static constexpr real_t MAX_RADS = 0.1745;

    /* The size of the interval is 2*MAX_YAW. */
    real_t size = 2 * MAX_RADS;

    /* Inner mod gives [0, 2*MAX_RADS) for positive numbers and (-2*MAX_RADS,0]
       for negative numbers. Adding MAX_RADS and modding by 2*MAX_RADS ensures
       that yaw+MAX_RADS maps to [0, 2*MAX_RADS). Subtracting MAX_RADS maps
       yaw to [-MAX_RADS, +MAX_RADS]. */
    real_t modYaw = fmod(fmod(yaw + MAX_RADS, size) + size, size) - MAX_RADS;

    /* Return the yaw jump. */
    return modYaw - yaw;
}

// Called by src-vivado with every IMU update after initialization/calibration
// is complete.
void updateMainFSM() {

    /* Previous flight mode initialized when the function is first called. */
    static FlightMode previousFlightMode = FlightMode::UNINITIALIZED;

    /* Values to be calculated each iteration. */
    AttitudeControlSignal uxyz;
    static real_t ucLast = 0.0; /* Remember last common thrust for GTC. */
    real_t uc            = ucLast;

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
    IMUMeasurement imuMeasurement = readIMU();
    Quaternion ahrsQuat           = updateAHRS(imuMeasurement);
    Quaternion jumpedAhrsQuat     = getJumpedOrientation(yawJump);

    /* Read sonar measurement and correct it using the drone's orientation. */
    bool hasNewSonarMeasurement      = readSonar();
    real_t sonarMeasurement          = getFilteredSonarMeasurement();
    real_t correctedSonarMeasurement = getCorrectedHeight(
        sonarMeasurement, attitudeController.getOrientationQuat());

    /* Read IMP measurement from shared memory and correct it using the sonar
       measurement and the drone's orientation. */
    Position positionMeasurement, correctedPositionMeasurement;
    real_t yawMeasurement;
    bool hasNewIMPMeasurement;
    if (visionComm->isDoneWriting()) {
        hasNewIMPMeasurement           = true;
        VisionData visionData          = visionComm->read();
        positionMeasurement            = visionData.position;
        yawMeasurement                 = visionData.yawAngle;
        ColVector<2> correctedPosition = getCorrectedPosition(
            ColVector<2>{positionMeasurement.x, positionMeasurement.y},
            sonarMeasurement, attitudeController.getOrientationQuat());
        correctedPositionMeasurement = {
            correctedPosition[0],
            correctedPosition[1]};  // TODO: adapter function
    }

    /* Implement main FSM logic. The transformed motor signals will be
       calculated based on the current flight mode, the measurements and the
       current state of the controllers. */
    switch (getFlightMode()) {
        case FlightMode::MANUAL: {

            /* Check whether the drone should be armed or disarmed. This should
               only occur in manual mode. */
            armedManager.update();

            /* Start gradual thrust change if last mode was altitude hold. */
            if (previousFlightMode == FlightMode::ALTITUDE_HOLD)
                gtcManager.start(ucLast);

            /* Common thrust comes directly from the RC, but leave some margin
               for two reasons. Firstly, it is not safe to fly at 100% thrust.
               Secondly, if that were to happen, then there would be no room for
               the attitude controller to make adjustments. */
            const float MAX_THROTTLE = 0.80;
            if (getThrottle() < 0.01) {
                uc = 0.0;
                attitudeController.init(); /* Reset attitude observer. */
                resetAHRSOrientation();    /* Reset AHRS to [1000]. */
            } else if (getThrottle() <= MAX_THROTTLE) {
                uc = getThrottle();
            } else {
                uc = MAX_THROTTLE;
            }

            /* Update the attitude controller's reference using the RC. */
            attitudeController.updateRCReference();

            /* Calculate the torque motor signals. */
            uxyz = attitudeController.updateControlSignal(uc);
        } break;
        case FlightMode::ALTITUDE_HOLD: {
            if (previousFlightMode == FlightMode::MANUAL)
                altitudeController.init();

            if (hasNewSonarMeasurement) {

                /* Update the altitude controller's reference using the RC. */
                altitudeController.updateRCReference();

                /* Common thrust is calculated by the altitude controller. */
                uc = inputBias.getThrustBias() +
                     altitudeController.updateControlSignal().ut;
            }
            /* Calculate the torque motor signals. */
            uxyz = attitudeController.updateControlSignal(uc);
        } break;
        case FlightMode::AUTONOMOUS: {
            // TODO: when should we initGround or initAir?
            if (previousFlightMode == FlightMode::ALTITUDE_HOLD)
                autonomousController.initAir(Position{0.5, 0.5},
                                             correctedSonarMeasurement);
            // TODO: should we tell IMP to reset their measurement?
            if (hasNewIMPMeasurement) {
                // TODO: in autonomous mode, update AHRS's yaw

                /* Autonomous controller, using position of the drone. */
                AutonomousOutput output =
                    autonomousController.update(correctedPositionMeasurement);

                /* Position controller, using AutonomousOutput. */
                PositionControlSignal q12ref;
                if (output.updatePositionController) {

                    /* Calculate current position state estimate. */
                    if (output.trustAccelerometerForPosition)
                        positionController.updateObserverBlind(
                            attitudeController.getOrientationQuat());

                    else
                        positionController.updateObserver(
                            attitudeController.getOrientationQuat(),
                            correctedPositionMeasurement);
                    // TODO: adapter

                    /* Calculate control signal. */
                    q12ref = positionController.updateControlSignal(
                        output.referencePosition);
                } else {
                    q12ref = {0.0, 0.0};
                }

                /* Altitude controller, using AutonomousOutput. */
                if (output.bypassAltitudeController) {
                    uc = output.commonThrust;
                } else {
                    altitudeController.setReference(output.referenceHeight);
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
                attitudeController.setReferenceEuler(quat1 + quat2);
                uxyz = attitudeController.updateControlSignal(uc);
            }
        } break;
        default:  // case FlightMode::UNINITIALIZED:
            /* We will never get here because readRC() cannot return an
            uninitialized flight mode. */
            uxyz = {0, 0, 0};
            uc   = 0.0;
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

    /* Transform the motor signals and output to the motors. */
    MotorSignals motorSignals = transformAttitudeControlSignal(uxyz, uc);
    if (armedManager.isArmed())
        outputMotorPWM(motorSignals);

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
    attitudeController.updateObserver({jumpedAhrsQuat, imuMeasurement.gx,
                                       imuMeasurement.gy, imuMeasurement.gz},
                                      yawJump);
    altitudeController.updateObserver(correctedSonarMeasurement);

    /* Logger. */
    setYawJump(yawJump);
    setIMUMeasurement(imuMeasurement);
    setAHRSQuat(ahrsQuat);
    setJumpedAHRSQuat(jumpedAhrsQuat);
    if (hasNewSonarMeasurement) {
        setSonarMeasurement(sonarMeasurement);
        setCorrectedSonarMeasurement(correctedSonarMeasurement);
    }
    if (hasNewIMPMeasurement) {
        setPositionMeasurement(positionMeasurement);
        setYawMeasurement(yawMeasurement);
        setCorrectedPositionMeasurement(correctedPositionMeasurement);
    }
    setCommonThrust(uc);
    setMotorSignals(motorSignals);

    /* Output log data if logger is done writing. */
    if (loggerComm->isDoneReading())
        loggerComm->write(getLogData());

    /* Store flight mode. */
    previousFlightMode = getFlightMode();
}

void updateFSM() {

    /* Generate heartbeat, so kill switch is activated when software hangs. */
    generateHeartbeat();

    /* Test pin high to probe length of interrupt. */
    writeValueToTestPin(true);

    /* Update LEDs. */
    writeToLEDs(isInterruptRunning, armedManager.isArmed(),
                getFlightMode() == FlightMode::AUTONOMOUS,
                getWPTMode() == WPTMode::ON);

    /* Set isInterruptRunning to true, mainLoop will set it to false. */
    isInterruptRunning = true;

    /* Update the drone's clock. */
    incrementTickCount();

    /* IMU bias should be calculated before use. */
    static bool isIMUCalibrated = false;
    /* AHRS should calibrate with accelerometer before use. */
    static bool isAHRSInitialized = false;

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
