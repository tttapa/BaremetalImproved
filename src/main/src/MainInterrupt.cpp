#include <output/Motors.hpp>
#include <platform/AxiGpio.hpp>
#include <sensors/AHRS.hpp>
#include <sensors/IMU.hpp>
#include <sensors/RC.hpp>
#include <sensors/Sonar.hpp>

#include <BaremetalCommunicationDef.hpp>
#include <ControllerInstances.hpp>
#include <MainInterrupt.hpp>
#include <MiscInstances.hpp>
#include <OutputValues.hpp>
#include <RCValues.hpp>
#include <SensorValues.hpp>
#include <SharedMemoryInstances.hpp>
#include <TiltCorrection.hpp>

#include <iostream>

/** Whether an interrupt is currently running. */
volatile bool isInterruptRunning = false;

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
                            {getCorrectedPositionMeasurement()});
                    // TODO: adapter

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
        printf("CALIBRATION MODE!!!");
        if (getThrottle() >= 0.50)
            uc = 1.0;
        else
            uc = 0.0;
    }

    /* Save the common thrust for the logger. */
    setCommonThrust(uc);

    /* Transform the motor signals and output to the motors. */
    MotorDutyCycles dutyCycles = transformAttitudeControlSignal(uxyz, uc);
    printf("Time %.2f\t, uc %.2f\t, ux %.2f\t, uy %.2f\t, uz %.2f, v0 %.2f\t, "
           "v1 %.2f\t, v2 %.2f\t, v3 %.2f\n",
           getTime(), uc, uxyz.ux, uxyz.uy, uxyz.uz, dutyCycles.v0,
           dutyCycles.v1, dutyCycles.v2, dutyCycles.v3);
    setDutyCycles(dutyCycles);
    if (armedManager.isArmed())
        outputMotorPWM(dutyCycles);

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

    /* Print. */
    /*printf("Time %f\t, thrust %f\t, yaw %f\t, pitch %f\t, roll %f\n", getTime(),
           getThrottle(), getYaw(), getPitch(), getRoll());*/
}

void updateFSM() {

    /* Test pin high to probe length of interrupt. */
    writeValueToTestPin(true);

    /* Update the drone's clock. */
    incrementTickCount();

    /* IMU bias should be calculated before use. */
    static bool isIMUCalibrated = false;
    /* AHRS should calibrate with accelerometer before use. */
    static bool isAHRSInitialized = false;

    /* Update LEDs. */
    writeToLEDs({isInterruptRunning, armedManager.isArmed(),
                 getFlightMode() == FlightMode::AUTONOMOUS,
                 getWPTMode() == WPTMode::ON});

    /* Set isInterruptRunning to true, mainLoop will set it to false. */
    isInterruptRunning = true;

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
