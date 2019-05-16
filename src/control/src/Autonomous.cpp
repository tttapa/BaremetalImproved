#include <Autonomous.hpp>

/* Includes from src. */
#include <BiasManager.hpp>
#include <ControllerInstances.hpp>  ///< PositionController correctPosition if drone gets lost during navigation
#include <MiscInstances.hpp>  ///< ESCStartupScript instance
#include <Position.hpp>
#include <RCValues.hpp>
#include <SharedMemoryInstances.hpp>
#include <Square.hpp>
#include <TestMode.hpp>
#include <Time.hpp>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>  ///< SECONDS_PER_TICK

#pragma region Constants
/**
 * If the drone is stays with 0.12 meters of its destination for a period of
 * time, then it will have converged on its target (horizontally).
 */
static constexpr real_t CONVERGENCE_DISTANCE_HORIZONTAL = 0.12;

/**
 * If the drone is stays with 0.05 meters of its reference height for a period
 * of time, then it will have converged on its target (vertically).
 */
static constexpr real_t CONVERGENCE_DISTANCE_VERTICAL = 0.05;

/**
 * If the drone is stays with a certain distance of its destination for 1
 * second, then it will have converged on its target.
 */
static constexpr real_t CONVERGENCE_DURATION = 1.0;

/**
 * The blind stage of the landing, meaning the sonar is not accurate anymore,
 * lasts 2.5 seconds.
 */
static constexpr real_t LANDING_BLIND_DURATION = 2.5;

/**
 * During the blind stage of the landing, meaning the sonar is not accurate
 * anymore, a marginal signal of 1% below the hovering signal will be sent to
 * the "common motor".
 */
static constexpr real_t LANDING_BLIND_MARGINAL_THRUST = -0.01;

/**
 * In the first stage of the landing procedure, when the sonar is accurate, the
 * reference height will decrease until it hits 0.25 meters.
 */
static constexpr real_t LANDING_LOWEST_REFERENCE_HEIGHT = 0.25;
;

/**
 * In the first stage of the landing procedure, when the sonar is accurate, the
 * reference height will decrease at a speed of 0.25 m/s until it hits the
 * minimum value, see getLandingLowestReferenceHeight().
 */
static constexpr real_t LANDING_REFERENCE_HEIGHT_SPEED = 0.25;

/**
 * If the autonomous controller is in the state LOITERING, NAVIGATING or
 * CONVERGING, then the drone will land if the throttle value goes below
 * 0.05.
 */
static constexpr real_t LANDING_THROTTLE = 0.05;

/** The autonomous controller will loiter for 15 seconds before navigating. */
static constexpr real_t LOITER_DURATION_LONG = 15.0;

/**
 * The autonomous controller will loiter for 3 seconds before navigating. This
 * is the case during the TEST_QR_WALKING mode.
 */
static constexpr real_t LOITER_DURATION_SHORT = 3.0;

/**
 * If the Cryptography team fails to decrypt the image sent by the Image
 * Processing team 3 times in a row, then we'll try reading it at a different
 * altitude. First, the images will be taken at the nominal reference height.
 * Then, they will be taken a little below the nominal value. After that, they
 * will be taken a little above the nominal value. Finally, images will be taken
 * at the nominal value again. If all of these images fail, the drone will give
 * up and either land or loiter indefinitely.
 */
static constexpr int MAX_QR_ERROR_COUNT = 3;

/**
 * If the Cryptography team fails to decrypt the image sent by the Image
 * Processing team too many times, then it's likely that the drone is not 
 * directly above the QR code. Therefore, it will start searching for it. After
 * 49 failed tiles (radius 3 around proposed QR position), the drone will stop
 * searching and attempt to land.
 */
static constexpr int MAX_QR_SEARCH_COUNT = 49;

/**
 * When the drone is navigating in autonomous mode, the reference will travel at
 * a speed of 0.5 m/s.
 */
static constexpr real_t NAVIGATION_SPEED = 0.5;

/** The pre-takeoff stage lasts 6.0 seconds. */
static constexpr real_t PRE_TAKEOFF_DURATION = 6.0;

/** During the pre-takeoff stage, the common thrust will be set to 0.20. */
static constexpr real_t PRE_TAKEOFF_COMMON_THRUST = 0.20;

/** The nominal reference height for the drone in autonomous mode is 1 meter. */
static constexpr real_t REFERENCE_HEIGHT = 1.0;

/**
 * If the QR code could not be read well, try taking images ± 0.15 meters from
 * the nominal reference height.
 */
static constexpr real_t REFERENCE_HEIGHT_ADJUSTMENT = 0.15;

/**
 * The blind stage of the takeoff, meaning the sonar is not yet accurate,
 * lasts 0.4 seconds.
 */
static constexpr real_t TAKEOFF_BLIND_DURATION = 0.4;

/**
 * During the blind stage of the takeoff, meaning the sonar is not yet accurate,
 * a marginal signal of 3% above the hovering signal will be sent to the "common
 * motor".
 */
static constexpr real_t TAKEOFF_BLIND_MARGINAL_THRUST = 0.03;

/** The entire takeoff will last 2 seconds. */
static constexpr real_t TAKEOFF_DURATION = 2.0;

/**
 * If the autonomous controller is in the state IDLE_GROUND, then the drone
 * will take off if the throttle value exceeds 0.50.
 */
static constexpr real_t TAKEOFF_THROTTLE = 0.50;
#pragma endregion

bool isValidSearchTarget(Position position) {
    return position[0] >= X_MIN && position[0] <= X_MAX &&
           position[1] >= Y_MIN && position[1] <= Y_MAX;
}

real_t AutonomousController::getElapsedTime() {
    return getTime() - this->autonomousStateStartTime;
}

Position AutonomousController::getNextSearchTarget() {
    real_t x = this->nextTarget[0];
    real_t y = this->nextTarget[1];

    /* Spiral outward until we reach the next tile to check. */
    real_t dx          = 1.0 * BLOCKS_TO_METERS;
    real_t dy          = 0.0 * BLOCKS_TO_METERS;
    int tilesUntilTurn = 0;
    int nextTurnIndex  = 1;
    for (int i = 0; i < this->qrTilesSearched; i++) {
        /* Every two iterations, the turn occurs 1 tile later. */
        if (i % 2 == 0)
            tilesUntilTurn++;
        /* Update dx, dy based on the index i. */
        if (i == nextTurnIndex) {
            dx = -dy;
            dy = dx;
            nextTurnIndex += tilesUntilTurn;
        }
        /* Next tile. */
        x += dx;
        y += dy;
    }
    return Position{x, y};
}

void AutonomousController::setAutonomousState(AutonomousState nextState) {
    this->autonomousState          = nextState;
    this->autonomousStateStartTime = getTime();
}

void AutonomousController::setNextTarget(Position target) {
    this->previousTarget = this->nextTarget;
    this->nextTarget     = target;
}

void AutonomousController::setNextTargetBlocks(Position targetBlocks) {
    setNextTarget(targetBlocks * BLOCKS_TO_METERS);
}

void AutonomousController::startNavigating(Position nextTarget) {
    setNextTarget(nextTarget);
    real_t d             = dist(this->previousTarget, this->nextTarget);
    this->navigationTime = d / NAVIGATION_SPEED;
    setAutonomousState(NAVIGATING);
}

void AutonomousController::startNavigatingBlocks(Position nextTargetBlocks) {
    startNavigating(nextTargetBlocks * BLOCKS_TO_METERS);
}

AutonomousOutput
AutonomousController::updateAutonomousFSM(Position currentPosition,
                                          real_t currentHeight) {

    /**
     * ============================ AUTONOMOUS FSM ============================
     * -------------- START ------------- | -------------- ERROR --------------
     * Idle_Ground: if drone is grounded  | Takeoff Stage 2 -> Error
     * Loitering: if drone is airborne    | Loitering -> Error
     *                                    | Converging -> Error
     * ----------- MAIN CYCLE ----------- | Navigating -> Error
     * Idle_Ground -> Pre_Takeoff         | Landing Stage 1 -> Error
     * Pre_Takeoff -> Takeoff             |
     * Takeoff -> Loitering               | ----- WIRELESS POWER TRANSFER -----
     * Loitering -> Converging            | Idle_Ground -> WPT
     * Converging <-> Navigating          | WPT -> Idle_Ground
     * Converging -> Landing              |
     * Landing -> Idle                    |
     *                                    |
     * -------- MID-FLIGHT CYCLE -------- |
     * Loitering -> Converging            |
     * Converging <-> Navigating          |
     * Converging -> Landing              |
     * Landing -> Idle                    |
     * -------------------------------------------------------------------------
     * 
     * !!! WHEN SWITCHING TO NAVIGATING, USE startNavigating(). DON'T USE    !!!
     * !!! setAutonomousState().                                             !!!
     * 
     */
    switch (this->autonomousState) {
        case IDLE_GROUND: return updateAutonomousFSM_IdleGround();
        case PRE_TAKEOFF: return updateAutonomousFSM_PreTakeoff();
        case TAKEOFF: return updateAutonomousFSM_Takeoff();
        case LOITERING: return updateAutonomousFSM_Loitering(currentPosition);
        case CONVERGING:
            return updateAutonomousFSM_Converging(currentPosition,
                                                  currentHeight);
        case NAVIGATING: return updateAutonomousFSM_Navigating(currentPosition);
        case LANDING: return updateAutonomousFSM_Landing();
        case WPT: return updateAutonomousFSM_WPT();
        case ERROR: return updateAutonomousFSM_Error();
        default:
            output = {false, {}, {}, false, false, {}, {}, false, false};
            return output;
    }
}

void AutonomousController::updateQRFSM() {

    /* Don't update QR FSM if the drone is not in CONVERGING state. */
    if (this->autonomousState != CONVERGING)
        return;

    /**
     * ================================ QR FSM ================================
     * --------------------------------- START --------------------------------
     * QR_Idle
     * 
     * ------------------------------ MAIN CYCLE ------------------------------
     *                                     |
     * ~~~~~~~~~~~~~~~ ANC ~~~~~~~~~~~~~~~ | ~~~~~~~~~~~~~~~ ANC ~~~~~~~~~~~~~~
     * QR_Idle -> QR_Reading               | QR_Error -> QR_Reading
     *                                     | QR_New_Target -> QR_Idle
     * ~~~~~~~~~~~~~~~ IMP ~~~~~~~~~~~~~~~ | QR_Land -> QR_Idle
     * QR_Reading -> QR_Crypto_Busy        | QR_Unknown -> QR_Idle
     *                                     |
     * ~~~~~~~~~~~~~~ CRYPTO ~~~~~~~~~~~~~ |
     * QR_Crypto_Busy -> QR_Error          |
     * QR_Crypto_Busy -> QR_New_Target     |
     * QR_Crypto_Busy -> QR_Land           |
     * QR_Crypto_Busy -> QR_Unknown        |
     * -------------------------------------------------------------------------
     */
    this->qrState = qrComm->getQRState();
    switch (this->qrState) {
        case QRFSMState::IDLE: return updateQRFSM_Idle();
        case QRFSMState::NEW_TARGET: return updateQRFSM_NewTarget();
        case QRFSMState::LAND: return updateQRFSM_Land();
        case QRFSMState::QR_UNKNOWN: return updateQRFSM_QRUnknown();
        case QRFSMState::NO_QR: return updateQRFSM_NoQR();
        case QRFSMState::ERROR: return updateQRFSM_Error();
        default:
            /* In any other case, it's not our job to update the QR FSM. It's up
               to either the Image Processing team or the Cryptography team. */
            return;
    }
}

void AutonomousController::initAir(Position currentPosition,
                                   AltitudeReference referenceHeight) {
    setAutonomousState(LOITERING);
    this->shouldLoiterIndefinitely = shouldLoiterIndefinitelyWithInitAir();
    this->previousTarget           = currentPosition;
    this->nextTarget               = currentPosition;
    this->referenceHeight          = referenceHeight;
    this->qrErrorCount             = 0;
}

void AutonomousController::initGround(Position currentPosition) {
    setAutonomousState(IDLE_GROUND);
    this->previousTarget = currentPosition;
    this->nextTarget     = currentPosition;
    this->qrErrorCount   = 0;
}

AutonomousOutput AutonomousController::update(Position currentPosition,
                                              real_t currentHeight) {
    updateQRFSM();
    return updateAutonomousFSM(currentPosition, currentHeight);
}

#pragma region Helper functions - QR FSM

void AutonomousController::updateQRFSM_Idle() {

    /* Reset error count and search count. */
    this->qrErrorCount    = 0;
    this->qrTilesSearched = 0;

    /* Switch autonomous FSM to NAVIGATING, and set the target to the next
       navigation test target if the convergence timer expires and we're in
       TEST_NAVIGATION mode. */
    if (getElapsedTime() > CONVERGENCE_DURATION &&
        isNavigationEnabledTestTargets())
        startNavigatingBlocks(getNextNavigationTestTarget());

    /* Set this FSM to QR_READ_REQUEST if the convergence timer expires and
       we're allowed to navigate with QR codes. */
    else if (getElapsedTime() > CONVERGENCE_DURATION &&
             isNavigationEnabledQRCodes())
        qrComm->setQRStateRequest();

    /* Otherwise, stay in QR_IDLE. */
}

void AutonomousController::updateQRFSM_NewTarget() {

    /* Correct the drone's position if it got lost during navigation. */
    positionController.correctPositionEstimateBlocks(
        qrComm->getCurrentPosition());

    /* Switch autonomous FSM to NAVIGATING, and set the target to the position
       of the next QR code sent by the Cryptography team. */
    startNavigatingBlocks(qrComm->getTargetPosition());

    /* Switch to QR_IDLE. */
    qrComm->setQRStateIdle();
}

void AutonomousController::updateQRFSM_Land() {

    /* Correct the drone's position if it got lost during navigation. */
    positionController.correctPositionEstimateBlocks(
        qrComm->getCurrentPosition());

    /* Switch the autonomous FSM to LANDING if landing is enabled. */
    if (isLandingEnabled())
        setAutonomousState(LANDING);

    /* Switch the autonomous FSM to LOITERING indefinitely is landing is
       disabled. */
    else {
        setAutonomousState(LOITERING);
        shouldLoiterIndefinitely = true;
    }

    /* Switch to QR_IDLE. */
    qrComm->setQRStateIdle();
}

void AutonomousController::updateQRFSM_QRUnknown() {

    /* Correct the drone's position if it got lost during navigation. */
    positionController.correctPositionEstimateBlocks(
        qrComm->getCurrentPosition());

    // TODO: what do we do with unknown QR data?

    /* Switch to QR_IDLE. */
    qrComm->setQRStateIdle();
}

void AutonomousController::updateQRFSM_NoQR() {

    /* Start (or continue) spiral-searching for QR code. */
    qrTilesSearched++;
    Position nextSearchTarget = getNextSearchTarget();
    while (!isValidSearchTarget(nextSearchTarget) &&
           qrTilesSearched <= MAX_QR_SEARCH_COUNT) {
        nextSearchTarget = getNextSearchTarget();
        qrTilesSearched++;
    }

    /* Set the autonomous FSM to NAVIGATING, and navigate to the next search
       target if we haven't run out of tiles to search. */
    if (qrTilesSearched <= MAX_QR_SEARCH_COUNT)
        startNavigating(nextSearchTarget);

    /* Set the autonomous FSM to LANDING if we've run out of tiles to search and
       landing is enabled. */
    else if (qrTilesSearched > MAX_QR_SEARCH_COUNT && isLandingEnabled())
        setAutonomousState(LANDING);

    /* Set the autonomous FSM to LOITERING indefinitely if we've run out of
       tiles to search and landing is disabled. */
    else if (qrTilesSearched > MAX_QR_SEARCH_COUNT && !isLandingEnabled()) {
        setAutonomousState(LOITERING);
        shouldLoiterIndefinitely = true;
    }

    /* Switch to QR_IDLE. */
    qrComm->setQRStateIdle();
}

void AutonomousController::updateQRFSM_Error() {

    /* Keep track of how many times we've failed reading the QR code. */
    qrErrorCount++;

    /* Increase the reference height a bit if we've failed 3 times. */
    if (qrErrorCount == 1 * MAX_QR_ERROR_COUNT) {
        referenceHeight.z = REFERENCE_HEIGHT + REFERENCE_HEIGHT_ADJUSTMENT;
        autonomousStateStartTime = getTime(); /* Reset convergence timer. */
        qrComm->setQRStateIdle();
        return;
    }

    /* Decrease the reference height a bit if we've failed 6 times. */
    if (qrErrorCount == 2 * MAX_QR_ERROR_COUNT) {
        referenceHeight.z = REFERENCE_HEIGHT + REFERENCE_HEIGHT_ADJUSTMENT;
        autonomousStateStartTime = getTime(); /* Reset convergence timer. */
        qrComm->setQRStateIdle();
        return;
    }

    /* Reset the reference height if we've failed 9 times. */
    if (qrErrorCount == 3 * MAX_QR_ERROR_COUNT) {
        referenceHeight.z        = REFERENCE_HEIGHT;
        autonomousStateStartTime = getTime(); /* Reset convergence timer. */
        qrComm->setQRStateIdle();
        return;
    }

    /* Switch to QR_READ_REQUEST if we've failed less than 12 times. */
    if (qrErrorCount < 4 * MAX_QR_ERROR_COUNT) {
        qrComm->setQRStateRequest();
        return;
    }

    /* Give up if we've failed 12 times. */
    /* Switch autonomous FSM to landing if landing is enabled. */
    if (isLandingEnabled())
        setAutonomousState(LANDING);

    /* Switch the autonomous FSM to LOITERING indefinitely is landing is
    disabled. */
    else {
        setAutonomousState(LOITERING);
        shouldLoiterIndefinitely = true;
    }

    /* Switch to QR_IDLE. */
    qrComm->setQRStateIdle();
}

#pragma endregion

#pragma region Helper functions - Autonomous FSM

AutonomousOutput AutonomousController::updateAutonomousFSM_IdleGround() {

    /* Switch to WPT when the RC WPT mode turns on. */
    if (getWPTMode() == WPTMode::ON)
        setAutonomousState(WPT);

    /* Switch to PRE_TAKEOFF when the RC throttle is raised high enough. */
    else if (getThrottle() > TAKEOFF_THROTTLE)
        setAutonomousState(PRE_TAKEOFF);

    /* Otherwise, stay in IDLE_GROUND. */
    output = AutonomousOutput{
        false,         // ✖ Don't use altitude controller
        {},            //   /
        {0.0},         //...bypass with zero common thrust
        false,         // ✖ Don't update altitude observer
        false,         // ✖ Don't use position controller
        {},            //   /
        {{0.0, 0.0}},  //...bypass with upright orientation
        false,         // ✖ Don't update position controller
        false,         //   /
    };
    return output;
}

AutonomousOutput AutonomousController::updateAutonomousFSM_PreTakeoff() {

    /* Switch to TAKEOFF if the timer expires and the test mode dictates that we
       should take off after pre-takeoff. */
    if (getElapsedTime() > PRE_TAKEOFF_DURATION &&
        shouldTakeOffAfterPreTakeoff())
        setAutonomousState(TAKEOFF);

    /* Switch back to IDLE if the timer expires and the test mode dictates that
       we should not take off after pre-takeoff. */
    else if (getElapsedTime() > PRE_TAKEOFF_DURATION &&
             !shouldTakeOffAfterPreTakeoff())
        setAutonomousState(IDLE_GROUND);

    /* Otherwise, stay in PRE_TAKEOFF. */
    output = AutonomousOutput{
        false,                        // ✖ Don't use altitude controller
        {},                           //   /
        {PRE_TAKEOFF_COMMON_THRUST},  //...bypass with pre-takeoff common thrust
        false,                        // ✖ Don't update altitude observer
        false,                        // ✖ Don't use position controller
        {},                           //   /
        {{0.0, 0.0}},                 //...bypass with upright orientation
        false,                        // ✖ Don't update position controller
        false,                        //   /
    };
    return output;
}

AutonomousOutput AutonomousController::updateAutonomousFSM_Takeoff() {

    /* Switch to LOITERING if the timer expires. */
    if (getElapsedTime() > TAKEOFF_DURATION) {
        setAutonomousState(LOITERING);
        this->shouldLoiterIndefinitely = shouldLoiterIndefinitelyAfterTakeoff();
    }

    /* Otherwise, stay in TAKEOFF. */
    /* Set the reference height to autonomous flight altitude. */
    referenceHeight = REFERENCE_HEIGHT;

    /* Stage 1: blind takeoff, so we can't trust the sonar or IMP. */
    if (getElapsedTime() < TAKEOFF_BLIND_DURATION)
        output = AutonomousOutput{
            false,  // ✖ Don't use altitude controller
            {},     //   /
                    //...bypass with the base hovering thrust plus a few percent
            {biasManager.getAutonomousHoveringThrust() +
             TAKEOFF_BLIND_MARGINAL_THRUST},
            false,       // ✖ Don't update altitude observer
            true,        // ✔ Use position controller
            nextTarget,  //...with current reference position
            {},          //   /
            true,        // ✔ Update position controller
            false,       //...trusting accelerometer
        };

    /* Stage 2: we can trust the sonar and IMP. */
    else
        output = AutonomousOutput{
            true,             // ✔ Use altitude controller
            referenceHeight,  //...with current reference height
            {},               //   /
            true,             // ✔ Update altitude observer
            true,             // ✔ Use position controller
            nextTarget,       //...with current reference position
            {},               //   /
            true,             // ✔ Update position controller
            true,             //...trusting IMP
        };

    return output;
}

AutonomousOutput
AutonomousController::updateAutonomousFSM_Loitering(Position currentPosition) {

    /* Determine if we should leave the LOITERING state. */
    bool timerCanExpire  = !this->shouldLoiterIndefinitely;
    bool timerHasExpired = (shouldLoiteringTimerBeShortened() &&
                            getElapsedTime() > LOITER_DURATION_SHORT) ||
                           getElapsedTime() > LOITER_DURATION_LONG;
    bool loiteringFinished = timerCanExpire && timerHasExpired;

    /* Switch to LANDING, and land at the current position, if landing is
       enabled and the pilot lowers the throttle enough. */
    if (isLandingEnabled() && getThrottle() <= LANDING_THROTTLE) {
        setAutonomousState(LANDING);
        setNextTarget(currentPosition);
    }

    /* Switch to LANDING, and land at the current position, if landing is
       enabled, loitering is finished and navigation is not enabled. */
    else if (isLandingEnabled() && loiteringFinished &&
             !isNavigationEnabled()) {
        setAutonomousState(LANDING);
        setNextTarget(currentPosition);
    }

    /* Switch to CONVERGING if loitering is finished and navigation is
       enabled. Also test QR search if the current test mode dictates it. */
    else if (loiteringFinished && isNavigationEnabled()) {
        setAutonomousState(CONVERGING);
        if (shouldTestQRSearch())
            positionController.correctPositionEstimateBlocks(
                positionController.getStateEstimate().p * METERS_TO_BLOCKS +
                Position{0.0, 1.0});
    }

    /* Otherwise, stay in LOITERING. */
    output = AutonomousOutput{
        true,             // ✔ Use altitude controller
        referenceHeight,  //...with current reference height
        {},               //   /
        true,             // ✔ Update altitude observer
        true,             // ✔ Use position controller
        nextTarget,       //...with current reference position
        {},               //   /
        true,             // ✔ Update position controller
        true,             //...trusting IMP
    };
    return output;
}

AutonomousOutput
AutonomousController::updateAutonomousFSM_Converging(Position currentPosition,
                                                     real_t currentHeight) {

    /* Switch to LANDING, and land at the current position, if landing is
       enabled and the pilot lowers the throttle enough. */
    if (isLandingEnabled() && getThrottle() <= LANDING_THROTTLE) {
        setAutonomousState(LANDING);
        setNextTarget(currentPosition);
    }

    /* If the QR FSM receives a NEW_TARGET instruction, then the autonomous
       controller's FSM will switch to NAVIGATING. */

    /* If the QR FSM receives a LAND instruction, then the autonomous
       controller's FSM will switch to LANDING (or it will loiter indefinitely
       if landing is not enabled). */

    /* If the QR FSM receives a NO_QR instruction, then the autonomous
       controller will start searching for the nearest QR code, so the FSM will
       switch to NAVIGATING. */

    /* If the QR FSM receives a ERROR instruction, then the autonomous
       controller will first adjust its altitude a few times. If it fails too
       many times, we'll give up with reading QR codes and try to land. So, if
       landing is enabled, the FSM will switch to LANDING; otherwise, it will
       loiter indefinitely. */

    /* Stay in CONVERGING until the QR FSM changes this FSM's state. Just reset
       the timer if we've exited the convergence distance. */
    real_t horizontalDistanceSq    = distsq(currentPosition, nextTarget);
    real_t maxHorizontalDistanceSq = sq(CONVERGENCE_DISTANCE_HORIZONTAL);
    real_t verticalDistance    = std::abs(currentHeight - referenceHeight.z);
    real_t maxVerticalDistance = CONVERGENCE_DISTANCE_VERTICAL;

    bool inHorizontalBounds = horizontalDistanceSq <= maxHorizontalDistanceSq;
    bool inVerticalBounds   = verticalDistance <= maxVerticalDistance;

    if (!inHorizontalBounds || !inVerticalBounds)
        this->autonomousStateStartTime = getTime(); /* Reset timer. */

    output = AutonomousOutput{
        true,             // ✔ Use altitude controller
        referenceHeight,  //...with current reference height
        {},               //   /
        true,             // ✔ Update altitude observer
        true,             // ✔ Use position controller
        nextTarget,       //...with current reference position
        {},               //   /
        true,             // ✔ Update position controller
        true,             //...trusting IMP
    };
    return output;
}

AutonomousOutput
AutonomousController::updateAutonomousFSM_Navigating(Position currentPosition) {

    /* Switch to LANDING, and land at the current position, if landing is
       enabled and the pilot lowers the throttle enough. */
    if (isLandingEnabled() && getThrottle() <= LANDING_THROTTLE) {
        setAutonomousState(LANDING);
        setNextTarget(currentPosition);
    }

    /* Switch to CONVERGING if the timer expires. */
    if (getElapsedTime() >= this->navigationTime) {
        setAutonomousState(CONVERGING);
    }

    /* Otherwise, stay in NAVIGATING. Interpolate the reference point between
       the last target and the next target during this time. */
    real_t factor          = getElapsedTime() / this->navigationTime;
    Position delta         = (nextTarget - previousTarget) * factor;
    Position interpolation = previousTarget + delta;

    output = AutonomousOutput{
        true,             // ✔ Use altitude controller
        referenceHeight,  //...with current reference height
        {},               //   /
        true,             // ✔ Update altitude observer
        true,             // ✔ Use position controller
        interpolation,    //...with interpolation point as reference
        {},               //   /
        true,             // ✔ Update position controller
        true,             //...trusting IMP
    };
    return output;
}

AutonomousOutput AutonomousController::updateAutonomousFSM_Landing() {

    /* Switch to IDLE_GROUND if the reference height is low enough and the timer
       expires. */
    if (referenceHeight.z <= LANDING_LOWEST_REFERENCE_HEIGHT &&
        getElapsedTime() > LANDING_BLIND_DURATION)
        setAutonomousState(IDLE_GROUND);

    /* Otherwise, stay in LANDING. */
    /* Stage 1: we can trust the sonar and IMP. Slowly lower the reference
                height until we reach the minimum value. */
    if (referenceHeight.z > LANDING_LOWEST_REFERENCE_HEIGHT) {
        referenceHeight.z -= LANDING_REFERENCE_HEIGHT_SPEED * SECONDS_PER_TICK;
        /* Reset timer: use it during stage 2 to know when to go to idle. */
        this->autonomousStateStartTime = getTime();
        output                         = AutonomousOutput{
            true,  // ✔ Use altitude controller
            referenceHeight,  //...with current reference height
            {},               //   /
            true,        // ✔ Update altitude observer
            true,        // ✔ Use position controller
            nextTarget,  //...with current reference position
            {},          //   /
            true,  // ✔ Update position controller
            true,  //...trusting IMP
        };
    }

    /* Stage 2: blind landing, so we can't trust the sonar or IMP. */
    else {
        output = AutonomousOutput{
            false,  // ✖ Don't use altitude controller
            {},     //   /
                    //...bypass with the actual hovering thrust minus a percent
            {biasManager.getThrustBias() + LANDING_BLIND_MARGINAL_THRUST},
            false,       // ✖ Don't update altitude observer
            true,        // ✔ Use position controller
            nextTarget,  //...with current reference position
            {},          //   /
            true,        // ✔ Update position controller
            false,       //...trusting accelerometer
        };
    }
    return output;
}

AutonomousOutput AutonomousController::updateAutonomousFSM_WPT() {

    /* Switch to IDLE_GROUND if the WPT is switched off. */
    if (getWPTMode() == WPTMode::OFF)
        setAutonomousState(IDLE_GROUND);

    /* Otherwise, stay in WPT. */
    output = AutonomousOutput{
        false,         // ✖ Don't use altitude controller
        {},            //   /
        {0.0},         //...bypass with zero common thrust
        false,         // ✖ Don't update altitude observer
        false,         // ✖ Don't use position controller
        {},            //   /
        {{0.0, 0.0}},  //...bypass with upright orientation
        false,         // ✖ Don't update position controller
        false,         //   /
    };
    return output;
}

AutonomousOutput AutonomousController::updateAutonomousFSM_Error() {

    /* Stay in ERROR until the pilot switches back to ALTITUDE-HOLD mode. */
    output = AutonomousOutput{
        true,             // ✔ Use altitude controller
        referenceHeight,  //...with current reference height
        {},               //   /
        true,             // ✔ Update altitude observer
        false,            // ✖ Don't use position controller
        {},               //   /
        {{0.0, 0.0}},     //...bypass with upright orientation
        false,            // ✖ Don't update position controller
        false,            //   /
    };
    return output;
}

#pragma endregion