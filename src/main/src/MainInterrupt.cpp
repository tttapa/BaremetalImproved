#include <MainInterrupt.hpp>
#include <iostream>

#pragma region Includes
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
#pragma endregion

/** Whether an interrupt is currently running. */
volatile bool isInterruptRunning = false;

/**
 * In manual flight mode, leave some thrust margin. There are two reasons for
 * this. Firstly, it is not safe to fly at 100% thrust. Secondly, if that were
 * to happen, then there would be no room for the attitude controller to make
 * adjustments.
 */
const real_t MAX_THROTTLE = 0.80;

// TODO: change drone mass!
// TODO: types degree, radian, meter, block
// TODO: where should this be?
const real_t BLOCKS_2_METERS = 0.30;

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
        buzzerManager.addInitiatedBeeps();
    }
    /* Phase 3: Main operation. */
    else {
        mainOperation();
    }

    /* Test pin low to probe length of interrupt. */
    writeValueToTestPin(false);
}

// Called by src-vivado with every IMU update after initialization/calibration
// is complete.
void mainOperation() {

    /* Previous flight mode initialized when the function is first called. */
    static FlightMode previousFlightMode = FlightMode::UNINITIALIZED;

    /* Values to be calculated each iteration. */
    AttitudeControlSignal uxyz;
    real_t uc;
    static real_t ucLast = 0.0; /* Remember for altitude & GTC. */
    static PositionControlSignal q12refLast = {}; /* Remember for autonomous. */

    /* Keep the attitude controller's state estimate near the unit quaternion
       [1;0;0;0] to ensure the stability of the control system. Whenever the yaw
       passes 10 degrees (0.1745 rad), it will jump to -10 degrees and vice
       versa. */
    real_t yawJump =
        calculateYawJump(attitudeController.getOrientationEuler().yaw);
    attitudeController.calculateJumpedQuaternions(yawJump);

#pragma region Read measurements
    /* Read RC data and update the global struct. */
    setRCInput(readRC());

    /* Read IMU measurement and update the AHRS. */
    // TODO: replace long function calls with e.g. readIMUAndUpdateAHRS();
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
    Position positionMeasurementBlocks, positionMeasurement,
        correctedPositionMeasurement;
    real_t yawMeasurement;
    bool hasNewIMPMeasurement = false;
    if (visionComm->isDoneWriting()) {
        hasNewIMPMeasurement      = true;
        VisionData visionData     = visionComm->read();
        positionMeasurementBlocks = visionData.position;
        positionMeasurement       = positionMeasurementBlocks * BLOCKS_2_METERS;
        yawMeasurement            = visionData.yawAngle;
        ColVector<2> correctedPosition = getCorrectedPosition(
            ColVector<2>{positionMeasurement.x, positionMeasurement.y},
            sonarMeasurement, attitudeController.getOrientationQuat());
        correctedPositionMeasurement = {
            correctedPosition[0],
            correctedPosition[1]};  // TODO: adapter function
    }
#pragma endregion

    /* Implement main FSM logic. The transformed motor signals will be
       calculated based on the current flight mode, the measurements and the
       current state of the controllers. */
    switch (getFlightMode()) {
        case FlightMode::MANUAL:
#pragma region Manual mode
        {
            /* Check whether the drone should be armed or disarmed. This should
               only occur in manual mode. */
            armedManager.update();

            /* Start gradual thrust change if last mode was altitude hold. */
            if (previousFlightMode == FlightMode::ALTITUDE_HOLD)
                gtcManager.start(ucLast);

            /* Common thrust comes directly from the RC, but leave margin. */
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
        }
#pragma endregion
        break;

        case FlightMode::ALTITUDE_HOLD:
#pragma region Altitude - hold mode
        {
            /* Initialize altitude controller if we switch from manual mode. */
            if (previousFlightMode == FlightMode::MANUAL) {
                altitudeController.init();
                altitudeController.setReference(correctedSonarMeasurement);
            }

            /* Update the altitude controller's reference using the RC. */
            altitudeController.updateRCReference();

            /* Update the altitude controller's signal at sonar frequency. */
            if (hasNewSonarMeasurement)
                uc = biasManager.getThrustBias() +
                     altitudeController.updateControlSignal().ut;
            else
                uc = ucLast;

            /* Calculate the torque motor signals. */
            uxyz = attitudeController.updateControlSignal(uc);
        }
#pragma endregion
        break;

        case FlightMode::AUTONOMOUS:
#pragma region Autonomous mode
        {
            // TODO: when should we initGround or initAir?
            if (previousFlightMode == FlightMode::ALTITUDE_HOLD)
                autonomousController.initAir(Position{0.5, 0.5},
                                             correctedSonarMeasurement);

            /* Update autonomous controller using most recent position. */
            AutonomousOutput output =
                autonomousController.update(getCorrectedPositionMeasurement());

            /* Calculate common thrust. */
            if (output.bypassAltitudeController) {
                uc = output.commonThrust;
            } else {
                altitudeController.setReference(output.referenceHeight);
                uc = biasManager.getThrustBias() +
                     altitudeController.updateControlSignal().ut;
            }

            /* Calculate reference orientation. */
            // TODO: if yaw turns, this should be different?
            PositionControlSignal q12ref;
            if (output.updatePositionController) {
                /* Blind position controller @ IMU frequency. */
                if (output.trustAccelerometerForPosition) {
                    positionController.updateObserverBlind(
                        attitudeController.getOrientationQuat());
                    q12ref = positionController.updateControlSignalBlind(
                        output.referencePosition);
                }
                /* Normal position controller @ IMP frequency. */
                else if (hasNewIMPMeasurement) {
                    positionController.updateObserver(
                        attitudeController.getOrientationQuat(),
                        correctedPositionMeasurement);
                    q12ref = positionController.updateControlSignal(
                        output.referencePosition);
                }
                /* Normal position controller should hold its previous control
                   signal while IMP has not sent a new measurement. */
                else {
                    q12ref = q12refLast;
                }
            } else {
                /* Keep reference upright if we're not supposed to update. */
                q12ref = {0.0, 0.0};
            }

            /* Set the attitude controller's reference orientation. We should
               add the input bias in order to center the position controller's
               control signal about the equilibrium. */
            Quaternion quatInputBias = EulerAngles::eul2quat({
                0.0,
                biasManager.getPitchBias(),
                biasManager.getRollBias(),
            });
            real_t q1                = q12ref.q1ref;
            real_t q2                = q12ref.q2ref;
            real_t q0                = 1 - sqrt(q1 * q1 + q2 * q2);
            Quaternion quatQ12Ref    = Quaternion(q0, q1, q2, 0);
            attitudeController.setReferenceEuler(quatQ12Ref + quatInputBias);

            /* Calculate the torque motor signals. */
            uxyz = attitudeController.updateControlSignal(uc);
        }
#pragma endregion
        break;

        default:
            /* We will never get here because readRC() cannot return an
            uninitialized flight mode. The compiler requires this, though. */
            uxyz = {0, 0, 0};
            uc   = 0.0;
            break;
    }

#pragma region Final thrust corrections(ESC, GTC, Calibration)
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
#pragma endregion

    /* Remember common thrust for the next clock cycle. */
    ucLast = uc;

    /* Transform the motor signals and output to the motors. */
    MotorSignals motorSignals = transformAttitudeControlSignal(uxyz, uc);
    if (armedManager.isArmed())
        outputMotorPWM(motorSignals);

    /* Update the Kalman Filters (the position controller doesn't use one). */
    attitudeController.updateObserver({jumpedAhrsQuat, imuMeasurement.gx,
                                       imuMeasurement.gy, imuMeasurement.gz},
                                      yawJump);
    altitudeController.updateObserver(correctedSonarMeasurement);

#pragma region Updates
    /* Update the controller configuration if the common thrust is near zero. */
    configManager.update(uc);

    /* Update the buzzer. */
    buzzerManager.update();

    // TODO: kill if the drone tilts too far? (droneControllersActivated flag)

    /* Update input biases. */
    // TODO: remember input bias so we can fly immediately in autonomous
    biasManager.updatePitchBias(attitudeController.getReferenceEuler().pitch,
                              getFlightMode());
    biasManager.updateRollBias(attitudeController.getReferenceEuler().roll,
                             getFlightMode());
    biasManager.updateThrustBias(uc, getFlightMode());
#pragma endregion

#pragma region Logger

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
        setPositionMeasurementBlocks(positionMeasurementBlocks);
        setPositionMeasurement(positionMeasurement);
        setYawMeasurement(yawMeasurement);
        setCorrectedPositionMeasurement(correctedPositionMeasurement);
    }
    setCommonThrust(uc);
    setMotorSignals(motorSignals);

    /* Output log data if logger is done writing. */
    if (loggerComm->isDoneReading())
        loggerComm->write(getLogData());
#pragma endregion

    /* Store flight mode. */
    previousFlightMode = getFlightMode();
}

#pragma region Helper functions
real_t calculateYawJump(real_t yaw) {

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
#pragma endregion