#include <MainInterrupt.hpp>
#include <iostream>

#pragma region Includes
/* Includes from src. */
#include <BaremetalCommunicationDef.hpp>
#include <ControllerInstances.hpp>
#include <LogEntry.h>
#include <MiscInstances.hpp>
#include <RCValues.hpp>
#include <SharedMemoryInstances.hpp>
#include <TestMode.hpp>
#include <TiltCorrection.hpp>
#include <Time.hpp>

/* Includes from src-vivado. */
#include <output/Motors.hpp>
#include <output/WPT.hpp>
#include <platform/AxiGpio.hpp>
#include <sensors/AHRS.hpp>
#include <sensors/IMU.hpp>
#include <sensors/RC.hpp>
#include <sensors/Sonar.hpp>
#pragma endregion

/** Whether an interrupt is currently running. */
volatile bool isInterruptRunning = false;

/** Flight mode from the previous cycle. */
static FlightMode previousFlightMode = FlightMode::UNINITIALIZED;

/** Current flight mode, depending on current test mode. */
static FlightMode flightMode = FlightMode::UNINITIALIZED;

/** Current WPT mode, depending on the current state of the drone. */
static WPTMode wptMode = WPTMode::OFF;

/**
 * In manual flight mode, leave some thrust margin. There are two reasons for
 * this. Firstly, it is not safe to fly at 100% thrust. Secondly, if that were
 * to happen, then there would be no room for the attitude controller to make
 * adjustments.
 */
const real_t MAX_THROTTLE = 0.80;

/**
 * If the throttle goes below 0.05, no thrust will be sent to the motors. In
 * this case, it is possible to turn on WPT in manual mode. It is also possible
 * to switch to AUTONOMOUS mode and take off from the ground if the throttle is
 * below this value (given that the drone has hovered once in ALTITUDE-HOLD
 * mode).
 */
const float MIN_THROTTLE = 0.05;

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
                flightMode == FlightMode::AUTONOMOUS, wptMode == WPTMode::ON);

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

#pragma region Read measurements
    /* Read RC data and update the global struct. */
    setRCInput(readRC());

    /* Read IMU measurement and update the AHRS. */
    // TODO: replace long function calls with e.g. readIMUAndUpdateAHRS();
    IMUMeasurement imuMeasurement = readIMU();
    Quaternion ahrsQuat           = updateAHRS(imuMeasurement);

    /* Read sonar measurement and correct it using the drone's orientation. */
    bool hasNewSonarMeasurement       = readSonar();
    bool shouldUpdateAltitudeObserver = hasNewSonarMeasurement;
    real_t sonarMeasurement           = getFilteredSonarMeasurement();
    real_t correctedSonarMeasurement  = getCorrectedHeight(
        sonarMeasurement, attitudeController.getOrientationQuat());

    /* Read IMP measurement from shared memory and correct it using the sonar
       measurement and the drone's orientation. */
    Position positionMeasurementBlocks, positionMeasurement,
        correctedPositionMeasurement, globalPositionEstimate;
    ColVector<2> correctedPositionVec, globalPositionEstimateVec;
    real_t yawMeasurement;
    bool hasNewIMPMeasurement = false;
    if (visionComm->isDoneWriting()) {
        hasNewIMPMeasurement      = true;
        VisionData visionData     = visionComm->read();
        positionMeasurementBlocks = visionData.position;
        positionMeasurement       = positionMeasurementBlocks * BLOCKS_2_METERS;
        yawMeasurement            = visionData.yawAngle;
        correctedPositionVec      = getCorrectedPosition(
            {positionMeasurement.x, positionMeasurement.y}, sonarMeasurement,
            attitudeController.getOrientationQuat());
        correctedPositionMeasurement = {correctedPositionVec[0],
                                        correctedPositionVec[1]};
    }
#pragma endregion

    /**
     * =========================================================================
     * ================================ MAIN FSM ===============================
     * =========================================================================
     * 
     * In the following code, the common thrust (uc) and the reference orientat-
     * ion (refEul) will be calculated based on the flight mode. Afterwards, the
     * reference orientation will be passed to the attitude controller in order
     * to calculate the transformed motor signals (uxyz). Then, uxyz and uc will
     * be transformed to MotorSignals, which will be sent to the four ESCs.
     * 
     * MANUAL:        - uc comes from RC throttle
     *                - refEul comes from RC pitch, roll, yaw
     * 
     * ALTITUDE-HOLD: - uc comes from the altitude controller
     *                - refEul comes from RC pitch, roll, yaw
     * 
     * AUTONOMOUS:    - uc comes from altitude controller or thrust is
     *                  overridden in blind stages of takeoff/landing
     *                - refEul comes from position controller, or is zero when
     *                  the autonomous drone is in its IDLE state.
     * 
     * =========================================================================
     */
    EulerAngles refEul;  // TODO: should this be like uc or upateAttitudeEuler?
    static real_t uc;

    /**
     * =========================================================================
     * ========================= FLIGHT MODES & TESTING ========================
     * =========================================================================
     * 
     * MANUAL flight mode is always enabled. However, ALTITUDE-HOLD mode and
     * certain aspects of AUTONOMOUS mode can be disabled. This is useful for
     * testing loitering, navigation, landing, etc. For more information about
     * how test modes affect the BareMetal framework, see src/misc/TestMode.hpp.
     * 
     * Step 1: The ANC team should first set the drone to TEST_MANUAL mode. In
     * this mode, only MANUAL mode is enabled: switching to any other mode has
     * no effect.
     * 
     * Step 2: Once the attitude controller is working, then the drone to be set
     * to TEST_ALTITUDE_HOLD mode. Now, if the pilot flips the flight mode
     * switch to the middle position, the drone will attempt to keep its current
     * altitude. The reference height can also be adjusted if the pilot raises
     * the throttle above 80%, and it can be lowered if he lowers it below 20%.
     * Flipping the flight mode switch to the final position has no effect.
     * 
     * Step 3: As soon as the altitude controller is well tuned, focus on
     * getting real-time localisation to work! This is by far the hardest part
     * of the project (trust me, I'm a former EAGLE member :p). While flying
     * in MANUAL or ALTITUDE-HOLD mode, you can plot the measurement position of
     * the drone. If the drone knows its position and doesn't jump squares, you
     * are ready to test LOITERING mode.
     * 
     * Step 4: Finally, the last position of the switch can be used. Set the
     * drone to TEST_LOITERING mode and switch to AUTONOMOUS mode when the drone
     * is stable hovering in ALTITUDE-HOLD mode. This test mode will keep the
     * autonomous controller in the LOITERING state indefinitely (until the
     * pilot switches back to ALTITUDE-HOLD mode, of course). Use this test mode
     * to fine tune your position controller.
     * 
     * Step 5: Verify the working of the autonomous navigation by setting the
     * drone to TEST_NAVIGATION mode. The autonomous controller will first
     * loiter for 15 seconds, then it will navigate to prespecified positions
     * in a loop. If you have a good position controller with a clamp of five
     * degrees or less, then this should be no problem.
     * 
     * Step 6: Test your landing procedure by switching to TEST_LANDING mode.
     * To do this, switch to AUTONOMOUS mode when the drone is stable hovering
     * in ALTITUDE-HOLD mode. This test mode will have the drone loiter for 15
     * seconds, then it will initiate the landing procedure. You can switch back
     * to MANUAL mode once the drone has landed.
     * 
     * Step 7: Verify the working of the QR codes by walking with the drone in
     * TEST_QR_WALKING mode (no propellers). "Hover" above the first QR code and
     * then switch to AUTONOMOUS mode. After "loitering" for 3 seconds, the
     * drone will try to decrypt the QR code three times. If it fails all three
     * times, it will set the reference height 15cm higher and try again. If
     * this also fails, do the same with a reference height 15cm lower than the
     * original reference height and try again. Finally, it will try again at
     * the original reference height. If no QR was decoded, then the controller
     * will give up and land.
     * 
     * If, on the other hand, the first QR code was decoded successfully, then
     * the position controller's reference position will shift from the first
     * QR code to the next QR code at a speed of 0.5m/s. As soon as the position
     * controller's estimated position matches the reference position (within
     * 10 cm), it will try to decrypt the next QR code. Repeat this until the
     * controller receives a QR_LAND instruction. Then the controller will
     * initiate its landing procedure.
     * 
     * After landing, switch back to MANUAL mode to reset it. If the drone gets
     * lost during navigation (if the position estimate jumps a square for
     * example), then the drone will attempt to correct its position using the
     * position stored in the QR codes. This will be tested in Step 8.
     * 
     * Step 8: If the drone gets lost during navigation, there is a mechanism in
     * place to correct the drone's position as soon as it can decode a QR code.
     * If, for example, the drone's position measurement jumps by 1 square
     * during navigation, then the drone will no longer converge to the proper
     * QR code. If this happens, the drone will perform a spiral search for the
     * nearest QR code (which contains the global position) and correct its
     * position estimate with this information.
     * 
     * Now, switch the drone to TEST_QR_NAVIGATION_LOST mode. During this test,
     * the drone will loiter for 15 seconds, then it will intentionally mess up
     * its position estimate by 1 square and perform a spiral search for the
     * first QR code. Just switch to AUTONOMOUS mode above the first QR code.
     * 
     * Step 9: Switch the drone to TEST_PRETAKEOFF mode. This time, keep the RC
     * throttle at zero and switch to AUTONOMOUS mode while the drone is
     * grounded. This will perform the pre-takeoff routine which can be used to
     * start up your own ESCs and to give the motors some intertia before taking
     * off. After this, the drone will return to the GROUND_IDLE state.
     * 
     * Step 10: Switch the drone to TEST_TAKEOFF mode. Again, keep the RC
     * throttle at zero and switch to AUTONOMOUS mode while the drone is
     * grounded. The drone will perform its pre-takeoff routine, then it will
     * take off and loiter until the pilot switches back to ALTITUDE-HOLD mode.
     * 
     * Step 11: Everything should be ready for the demo. Switch the drone to
     * DEMO mode. The pilot can finally switch to AUTONOMOUS mode from either
     * the ground (RC throttle is zero) or from the air (RC throttle is not
     * zero). Make sure in either case that AUTONOMOUS mode is initialized above
     * the first QR code. If it starts from the ground, the drone will perform
     * its pre-takeoff and takeoff routines. Then, (in both cases) it will
     * loiter for 15 seconds. After that, it will follow the QR trail and land
     * at the final QR code. Finally, the WPT switch can be used to start
     * wireless power transfer.
     * 
     * Note that wireless power transfer can also be used in MANUAL mode if the
     * RC throttle is zero.
     * 
     * =========================================================================
     */

    /* First, set the actual flight mode based on which tests are allowed to
       be run. */
    flightMode = getFlightMode();
    if (flightMode == FlightMode::AUTONOMOUS && !canStartAutonomousMode())
        flightMode = FlightMode::ALTITUDE_HOLD;
    if (flightMode == FlightMode::ALTITUDE_HOLD && !canStartAltitudeHoldMode())
        flightMode = FlightMode::MANUAL;

    /* If we've just exited ALTITUDE-HOLD mode, set the autonomous controller's
       hovering thrust (only has effect if hovering thrust is >= 0.30, see
       BiasManager.hpp). */
    if (previousFlightMode == FlightMode::ALTITUDE_HOLD)
        biasManager.setAutonomousHoveringThrust(biasManager.getThrustBias());

    /* Also, set the actual WPT mode based on the current state of the drone.
       WPT can be turned on from MANUAL mode (zero thrust) or from AUTONOMOUS
       mode if the drone is grounded.*/
    bool wptManualMode =
        (flightMode == FlightMode::MANUAL && getThrottle() <= MIN_THROTTLE);
    bool wptAutonomousMode = (flightMode == FlightMode::AUTONOMOUS &&
                              autonomousController.getAutonomousState() == WPT);
    if (getWPTMode() == WPTMode::ON && (wptManualMode || wptAutonomousMode))
        wptMode = WPTMode::ON;
    else
        wptMode = WPTMode::OFF;

    /**
     * =========================================================================
     * =========================== MANUAL FLIGHT MODE ==========================
     * =========================================================================
     * 
     * As mentioned above, uc comes directly from the RC throttle. The reference
     * roll and pitch also come directly from the RC. The RC yaw will be used to
     * control how fast the reference yaw changes.
     * 
     * =========================================================================
     * !!! MANUAL mode is always enabled. If the mode switches to ALTITUDE-  !!!
     * !!! HOLD mode or AUTONOMOUS mode, but the altitude controller is not  !!!
     * !!! ready to test (see src/misc/TestMode.hpp), then the flight mode   !!!
     * !!! will remain in MANUAL mode.                                       !!!
     * =========================================================================
     */
    if (flightMode == FlightMode::MANUAL) {

#pragma region Manual mode

        //=========================== MISCELLANEOUS ==========================//

        /* Check whether the drone should be armed or disarmed. This should
               only occur in manual mode. */
        armedManager.update();

        /* Start gradual thrust change if last mode was altitude hold. */
        if (previousFlightMode == FlightMode::ALTITUDE_HOLD)
            gtcManager.start(uc);  // Previous value of uc...

        //=========================== COMMON THRUST ==========================//

        /* Common thrust comes directly from the RC, but leave margin. */
        if (getThrottle() < MIN_THROTTLE) {
            uc = 0.0;
            attitudeController.init(); /* Reset attitude observer. */
            resetAHRSOrientation();    /* Reset AHRS to [1000]. */
        } else if (getThrottle() <= MAX_THROTTLE) {
            uc = getThrottle();
        } else {
            uc = MAX_THROTTLE;
        }

        //======================= REFERENCE ORIENTATION ======================//

        /* Update the attitude controller's reference using the RC. */
        attitudeController.updateRCReference();

#pragma endregion
    }

    /**
     * =========================================================================
     * ======================= ALTITUDE-HOLD FLIGHT MODE =======================
     * =========================================================================
     * 
     * As mentioned above, uc comes from the altitude controller, whose height
     * can be adjusted using the RC throttle. The reference roll and pitch still
     * come directly from the RC pitch and roll. The RC yaw will be used to
     * control how fast the reference yaw changes.
     * 
     * =========================================================================
     * !!! ALTITUDE-HOLD mode is enabled in every mode but TEST_MANUAL. In   !!!
     * !!! all other modes, switching to AUTONOMOUS mode has no effect: the  !!!
     * !!! drone will remain in ALTITUDE-HOLD mode.                          !!!
     * =========================================================================
     */
    if (flightMode == FlightMode::ALTITUDE_HOLD) {

#pragma region Altitude - hold mode

        //========================== INITIALIZATION ==========================//

        /* Initialize altitude controller to track the current corrected sonar
           measurement if we switch from manual mode. */
        if (previousFlightMode == FlightMode::MANUAL)
            altitudeController.init(correctedSonarMeasurement);

        //=========================== MISCELLANEOUS ==========================//

        /* Update the altitude controller's reference using the RC. */
        altitudeController.updateRCReference();

        //=========================== COMMON THRUST ==========================//

        /* Update the altitude controller's signal at sonar frequency. */
        if (hasNewSonarMeasurement)
            uc = biasManager.getThrustBias() +
                 altitudeController.updateControlSignal().ut;

        //======================= REFERENCE ORIENTATION ======================//

        /* Update the attitude controller's reference using the RC. */
        attitudeController.updateRCReference();

#pragma endregion
    }

    /**
     * =========================================================================
     * ========================= AUTONOMOUS FLIGHT MODE ========================
     * =========================================================================
     * 
     * The calculation of the common thrust and the reference orientation is
     * much more complex in autonomous mode. The common thrust can either be
     * directly given by the autonomous controller (for example if the drone is
     * too low to the ground, and the sonar cannot be trusted) or it is determi-
     * ned by the altitude controller. Similarly for the reference orientation,
     * the autonomous controller can send a value directly to the attitude
     * controller in order to bypass the position controller. Otherwise the
     * reference orientation is calculated by the position controller and the 
     * anti-yaw-drift controller.
     * 
     * =========================================================================
     * !!! AUTONOMOUS mode is enabled from the air in the following test     !!!
     * !!! modes: TEST_LOITERING, TEST_NAVIGATION, TEST_LANDING, TEST_QR_WAL-!!!
     * !!! KING, TEST_QR_NAVIGATION_LOST and DEMO. AUTONOMOUS mode is enabled!!!
     * !!! from the ground in the following test modes: TEST_PRETAKEOFF,     !!!
     * !!! TEST_TAKEOFF, DEMO.                                               !!!
     * =========================================================================
     */
    if (flightMode == FlightMode::AUTONOMOUS) {

#pragma region Autonomous mode

        //========================== INITIALIZATION ==========================//

        if (previousFlightMode == FlightMode::ALTITUDE_HOLD) {
            if (canStartAutonomousModeAir())
                autonomousController.initAir(correctedPositionMeasurement,
                                             correctedSonarMeasurement);
            else
                autonomousController.initGround(correctedPositionMeasurement);
            positionController.init(correctedPositionMeasurement);
        }

        //=========================== MISCELLANEOUS ==========================//

        /* Calculate global position estimate. */
        globalPositionEstimateVec = getGlobalPositionEstimate(
            correctedPositionVec, positionController.getStateEstimate(),
            getTime() - positionController.getLastMeasurementTime());
        globalPositionEstimate = {globalPositionEstimateVec[0],
                                  globalPositionEstimateVec[1]};

        /* Update autonomous controller using most recent position. */
        AutonomousOutput output = autonomousController.update(
            globalPositionEstimate, correctedSonarMeasurement);

        /* Update altitude observer? */
        shouldUpdateAltitudeObserver = output.updateAltitudeObserver;

        /* Update position observer? */
        if (output.updatePositionObserver) {
            if (output.trustIMPForPosition) { /* Blind @ IMU frequency */
                positionController.updateObserverBlind(
                    attitudeController.getOrientationQuat());
            } else if (hasNewIMPMeasurement) { /* Normal @ IMP frequency */
                positionController.updateObserver(
                    attitudeController.getOrientationQuat(),
                    globalPositionEstimate);
            }
        }

        //=========================== COMMON THRUST ==========================//

        if (output.useAltitudeController) {
            altitudeController.setReference(output.referenceHeight);
            uc = biasManager.getThrustBias() +
                 altitudeController.updateControlSignal().ut;
        } else {
            uc = output.commonThrust.ut;
        }

        //======================= REFERENCE ORIENTATION ======================//

        // TODO: if yaw turns, this should be different?
        static PositionControlSignal q12ref;

        /* Use position controller. */
        if (output.usePositionController) {
            if (!output.trustIMPForPosition) { /* Blind @ IMU frequency */
                q12ref = positionController.updateControlSignalBlind(
                    output.referencePosition);
                `
            } else if (hasNewIMPMeasurement) { /* Normal @ IMP frequency */
                q12ref = positionController.updateControlSignal(
                    output.referencePosition);
            }
            /* Normal position controller should hold its previous control
               signal while IMP has not sent a new measurement. */
        }

        /* Bypass position controller. */
        else
            q12ref = output.q12ref;

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
        attitudeController.setReferenceEuler(quatInputBias + quatQ12Ref);

#pragma endregion
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

    /* Wireless power transfer? */
    if (wptMode == WPTMode::ON) {
        outputWPT((getTuner() + 1.0) / 2.0); /* Duty cycle: 0% to 100%. */
        uc = 0.0;
    }

#pragma endregion

    /* Keep the attitude controller's state estimate near the unit quaternion
       [1;0;0;0] to ensure the stability of the control system. Whenever the yaw
       passes 10 degrees (0.1745 rad), it will jump to -10 degrees and vice
       versa. */
    real_t yawJump =
        calculateYawJump(attitudeController.getOrientationEuler().yaw);
    attitudeController.calculateJumpedQuaternions(yawJump);

    /* Calculate the torque motor signals. The attitude controller's reference
       orientation has already been updated in the code above. */
    AttitudeControlSignal uxyz = attitudeController.updateControlSignal(uc);

    /* Transform the motor signals and output to the motors. */
    MotorSignals motorSignals = transformAttitudeControlSignal(uxyz, uc);
    if (armedManager.isArmed())
        outputMotorPWM(motorSignals);

    /* Update the Kalman Filters (the position controller doesn't use one). */
    Quaternion jumpedAhrsQuat = getAHRSJumpedOrientation(yawJump);
    attitudeController.updateObserver({jumpedAhrsQuat, imuMeasurement.gx,
                                       imuMeasurement.gy, imuMeasurement.gz},
                                      yawJump);
    if (shouldUpdateAltitudeObserver)
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
                                flightMode);
    biasManager.updateRollBias(attitudeController.getReferenceEuler().roll,
                               flightMode);
    biasManager.updateThrustBias(uc, flightMode);
#pragma endregion

#pragma region Logger

    /* Logger. */

    LogEntry logEntry;
    logEntry.setSize(64);
    logEntry.setMode(int32_t(flightMode));
    logEntry.setFrametime(getMillis());
    logEntry.setFramecounter(getTickCount());
    logEntry.setDroneConfig(configManager.getControllerConfiguration());
    logEntry.setRcTuning(getTuner());
    logEntry.setRcThrottle(getThrottle());
    logEntry.setRcRoll(getRoll());
    logEntry.setRcPitch(getPitch());
    logEntry.setRcYaw(getYaw());
    logEntry.setReferenceOrientation(
        toCppArray(attitudeController.getReferenceQuat()));
    logEntry.setReferenceOrientationEuler(
        toCppArray(attitudeController.getReferenceEuler()));
    logEntry.set__pad0(0);
    logEntry.setReferenceHeight(altitudeController.getReferenceHeight());
    logEntry.setReferenceLocation(
        toCppArray(positionController.getReferencePosition()));
    logEntry.setMeasurementOrientation(toCppArray(ahrsQuat));
    logEntry.setMeasurementAngularVelocity(
        toCppArray(GyroMeasurement{imuMeasurement}));
    logEntry.setMeasurementHeight(correctedSonarMeasurement);
    logEntry.setMeasurementLocation(toCppArray(globalPositionEstimate));
    logEntry.setAttitudeObserverState(
        toCppArray(attitudeController.getStateEstimate()));
    logEntry.setAltitudeObserverState(
        toCppArray(altitudeController.getStateEstimate()));
    logEntry.setNavigationObserverState(
        toCppArray(positionController.getStateEstimate()));
    logEntry.setAttitudeYawOffset(yawJump);
    logEntry.setAttitudeControlSignals(
        toCppArray(attitudeController.getControlSignal()));
    logEntry.setAltitudeControlSignal(altitudeController.getControlSignal().ut);
    logEntry.setPositionControlSignal(
        toCppArray(positionController.getControlSignal()));
    logEntry.setMotorControlSignals(toCppArray(motorSignals));
    logEntry.setCommonThrust(uc);
    logEntry.setHoverThrust(biasManager.getThrustBias());

    // TODO:
    (void) yawMeasurement;

    // logEntry.setSize(64);
    // logEntry.setMode(int32_t(getFlightMode()));
    // logEntry.setFrametime(getMillis());
    // logEntry.setFramecounter(getTickCount());
    // logEntry.setDroneConfig(configManager.getControllerConfiguration());
    // logEntry.setRcTuning(getTuner());
    // logEntry.setRcThrottle(getThrottle());
    // logEntry.setRcRoll(getRoll());
    // logEntry.setRcPitch(getPitch());
    // logEntry.setRcYaw(getYaw());
    // logEntry.set__pad0(0);
    // logEntry.setReferenceHeight(altitudeController.getReferenceHeight());
    // logEntry.setMeasurementHeight(correctedSonarMeasurement);
    // logEntry.setAttitudeYawOffset(yawJump);
    // logEntry.setAltitudeControlSignal(altitudeController.getControlSignal().ut);
    // logEntry.setPositionControlSignal(
    //     {(float)positionController.getControlSignal().q1ref,
    //      (float)positionController.getControlSignal().q2ref});
    // logEntry.setCommonThrust(uc);
    // logEntry.setHoverThrust(biasManager.getThrustBias());

    // // TODO:
    // (void) yawMeasurement;
    // (void) ahrsQuat;

    //setYawJump(yawJump);
    //setIMUMeasurement(imuMeasurement);
    //setAHRSQuat(ahrsQuat);
    //setJumpedAHRSQuat(jumpedAhrsQuat);
    //if (hasNewSonarMeasurement) {
    //    setSonarMeasurement(sonarMeasurement);
    //    setCorrectedSonarMeasurement(correctedSonarMeasurement);
    //}
    //if (hasNewIMPMeasurement) {
    //    setPositionMeasurementBlocks(positionMeasurementBlocks);
    //    setPositionMeasurement(positionMeasurement);
    //    setYawMeasurement(yawMeasurement);
    //    setCorrectedPositionMeasurement(correctedPositionMeasurement);
    //}
    //setCommonThrust(uc);
    //setMotorSignals(motorSignals);

    /* Output log data if logger is done writing. */

    if (loggerComm->isDoneReading())
        loggerComm->write(logEntry);

#pragma endregion

    /* Store flight mode. */
    previousFlightMode = flightMode;
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

template <class ArrayElementType = float, class StructType = void>
static ArrayElementType (&toCppArray(
    StructType &data))[sizeof(StructType) / sizeof(ArrayElementType)] {
    static_assert(sizeof(StructType) % sizeof(ArrayElementType) == 0);
    return reinterpret_cast<
        ArrayElementType(&)[sizeof(StructType) / sizeof(ArrayElementType)]>(
        data);
}

template <class ArrayElementType = float, class StructType = void>
static const ArrayElementType (&toCppArray(
    const StructType &data))[sizeof(StructType) / sizeof(ArrayElementType)] {
    static_assert(sizeof(StructType) % sizeof(ArrayElementType) == 0);
    return reinterpret_cast<const ArrayElementType(
            &)[sizeof(StructType) / sizeof(ArrayElementType)]>(data);
}

#pragma endregion