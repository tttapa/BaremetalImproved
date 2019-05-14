#include <Autonomous.hpp>

/* Includes from src. */
#include <BiasManager.hpp>
#include <ControllerInstances.hpp>  ///< PositionController correctPosition if drone gets lost during navigation
#include <MiscInstances.hpp>  ///< ESCStartupScript instance
#include <Position.hpp>
#include <RCValues.hpp>
#include <SharedMemoryInstances.hpp>
#include <TestMode.hpp>
#include <Time.hpp>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>  ///< SECONDS_PER_TICK

#pragma region Constants
/**
 * If the drone is stays with 0.10 meters of its destination for a period of
 * time, then it will have converged on its target.
 */
static constexpr real_t CONVERGENCE_DISTANCE = 0.10;

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
static constexpr real_t LANDING_REFERENCE_HEIGHT_DECREASE_SPEED = 0.25;

/**
 * If the autonomous controller is in the state LOITERING, NAVIGATING or
 * CONVERGING, then the drone will land if the throttle value goes below
 * 0.05.
 */
static constexpr real_t LANDING_THROTTLE = 0.05;

/**
 * The autonomous controller will loiter for 15 seconds before navigating.
 */
static constexpr real_t LOITER_DURATION = 15.0;

/**
 * If the Cryptography team fails to decrypt the image sent by the Image
 * Processing team 3 times in a row, then it's likely that the drone is not
 * directly above the QR code. Therefore, it will start searching for it.
 */
static constexpr int MAX_QR_ERROR_COUNT = 3;

/**
 * If the Cryptography team fails to decrypt the image sent by the Image
 * Processing team too many times, then it's likely that the drone is not 
 * directly above the QR code. Therefore, it will start searching for it. After
 * 25 failed tiles (radius 2 around proposed QR position), the drone will stop
 * searching and attempt to land.
 */
static constexpr int MAX_QR_SEARCH_COUNT = 25;

/**
 * When the drone is navigating in autonomous mode, the reference will travel at
 * a speed of 0.5 m/s.
 */
static constexpr real_t NAVIGATION_SPEED = 0.5;

/** The pre-takeoff stage lasts 6.0 seconds. */
static constexpr real_t PRE_TAKEOFF_DURATION = 6.0;

/** During the pre-takeoff stage, the common thrust will be set to 0.20. */
static constexpr real_t PRE_TAKEOFF_COMMON_THRUST = 0.20;

/** The normal reference height for the drone in autonomous mode is 1 meter. */
static constexpr real_t REFERENCE_HEIGHT = 1.0;

/**
 * The blind stage of the takeoff, meaning the sonar is not yet accurate,
 * lasts 0.5 seconds.
 */
static constexpr real_t TAKEOFF_BLIND_DURATION = 0.5;

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
    return position.x >= X_MIN && position.x <= X_MAX && position.y >= Y_MIN &&
           position.y <= Y_MAX;
}

real_t AutonomousController::getElapsedTime() {
    return getTime() - this->autonomousStateStartTime;
}

Position AutonomousController::getNextSearchTarget() {
    real_t x = this->nextQRPosition.x;
    real_t y = this->nextQRPosition.y;

    /* Spiral outward until we reach the next tile to check. */
    real_t dx          = 1.0;
    real_t dy          = 0.0;
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
    return Position(x, y);
}

void AutonomousController::setAutonomousState(AutonomousState nextState) {
    this->autonomousState          = nextState;
    this->autonomousStateStartTime = getTime();
}

void AutonomousController::setNextTarget(Position target) {
    this->previousTarget = this->nextTarget;
    this->nextTarget     = target;
}

void AutonomousController::startLanding(bool shouldLandAtCurrentPosition,
                                        Position currentPosition) {
    if (shouldLandAtCurrentPosition)
        setNextTarget(currentPosition);
    setAutonomousState(LANDING);
}

void AutonomousController::startNavigating(Position nextQRPosition) {
    setNextTarget(nextQRPosition);
    real_t d             = dist(this->previousTarget, this->nextTarget);
    this->navigationTime = d / NAVIGATION_SPEED;
    setAutonomousState(NAVIGATING);
}

void AutonomousController::updateQRFSM() {

    /* Load the QR state from shared memory. */
    this->qrState = qrComm->getQRState();

    /* Don't update QR FSM if the drone is not in CONVERGING state. */
    if (this->autonomousState != CONVERGING)
        return;

#pragma region QRFSM

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
     */

    /* Implement FSM logic. */
    real_t correctionX, correctionY;
    switch (this->qrState) {
        case QRFSMState::IDLE:
            /* Let the Image Processing team take a picture if we have converged
               on our target. */
            if (getElapsedTime() > CONVERGENCE_DURATION) {
                /* Test navigation: set new target. */
                if(isNavigatingEnabled())
                    startNavigating(getNextNavigationTestTarget());
                /* Get new target from QR code. */
                else if(isQRReadingEnabled())
                    qrComm->setQRStateRequest();
            }
                
            break;
        case QRFSMState::NEW_TARGET:
            /* Reset error count and search count. */
            this->qrErrorCount    = 0;
            this->qrTilesSearched = 0;

            /* Correct the drone's position if the drone got lost, and we had to
               search for the code. */
            correctionX = this->nextTarget.x - this->nextQRPosition.x;
            correctionY = this->nextTarget.y - this->nextQRPosition.y;
            if (correctionX != 0.0 && correctionY != 0.0)
                positionController.correctPosition(correctionX, correctionY);

            /* Tell the autonomous controller's FSM to start navigating to the
               position of the next QR code sent by the Cryptography team. */
            /* Switch this FSM to QR_IDLE. */
            startNavigating(qrComm->getTargetPosition());
            qrComm->setQRStateIdle();
            break;
        case QRFSMState::LAND:
            /* Reset error count and search count. */
            this->qrErrorCount                 = 0;
            this->qrTilesSearched              = 0;
            this->hasReceivedQRLandInstruction = true;

            /* Tell the autonomous controller's FSM to start landing. */
            /* Switch this FSM to QR_IDLE. */
            if (isLandingEnabled())
                startLanding(false, {});
            else
                setAutonomousState(LOITERING);

            qrComm->setQRStateIdle();
            break;
        case QRFSMState::QR_UNKNOWN:
            /* Reset error count and search count. */
            this->qrErrorCount    = 0;
            this->qrTilesSearched = 0;

            // TODO: what do we do with unknown QR data?

            /* Switch this FSM to QR_IDLE. */
            qrComm->setQRStateIdle();
            break;
        case QRFSMState::ERROR:
            // TODO: instead of trying 3 times, check if there's a QR Code
            // TODO: if QR code, then try up to 15 times, then quit (land)
            // TODO: if no QR code (3 times?), then start searching
            this->qrErrorCount++;
            if (this->qrErrorCount <= MAX_QR_ERROR_COUNT) {
                /* Tell IMP to try again. */
                qrComm->setQRStateRequest();
            } else {

                /* Start (or continue) spiral-searching for QR code. */
                this->qrErrorCount = 0;
                this->qrTilesSearched++;
                Position nextSearchTarget = getNextSearchTarget();
                while (!isValidSearchTarget(nextSearchTarget) &&
                       this->qrTilesSearched < MAX_QR_SEARCH_COUNT) {
                    nextSearchTarget = getNextSearchTarget();
                    this->qrTilesSearched++;
                }

                /* Valid search target, so set it as the next target. */
                /* Switch this FSM to QR_IDLE. */
                if (this->qrTilesSearched < MAX_QR_SEARCH_COUNT) {
                    setNextTarget(nextSearchTarget);
                    qrComm->setQRStateIdle();
                }

                /* We've run out of tiles to search, so have the drone land. */
                /* Switch this FSM to QR_IDLE. */
                else {
                    // TODO: fix this, we shouldn't land if not enabled
                    startLanding(false, {});
                    qrComm->setQRStateIdle();
                }
            }
            break;
        default:
            /* In any other case, it's not our job to update the QR FSM. It's up
               to either the Image Processing team or the Cryptography team. */
            break;
    }

#pragma endregion
}

AutonomousOutput
AutonomousController::updateAutonomousFSM(Position currentPosition) {

#pragma region AutonomousFSM
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
     */

    /* Default values for the autonomous controller's output... Instruction =
       hover at (position, height) = (nextTarget, referenceHeight). */
    bool bypassAltitudeController      = false;
    real_t commonThrust                = 0.0;
    bool updatePositionController      = true;
    bool trustAccelerometerForPosition = false;
    Position referencePosition         = this->nextTarget;

    /* Implement FSM logic. */
    real_t dx, dy;
    switch (this->autonomousState) {

        case IDLE_GROUND:
            /* Instruction: override thrust (no thrust), don't update position
                            controller.
               Switch to PRE_TAKEOFF: when throttle is raised high enough. */
            bypassAltitudeController = true;
            commonThrust             = 0.0;
            updatePositionController = false;
            if (getThrottle() > TAKEOFF_THROTTLE)
                setAutonomousState(PRE_TAKEOFF);
            break;

        case PRE_TAKEOFF:
            /* Instruction: override thrust (pre-takeoff thrust), don't update
                            position.
               Switch to TAKEOFF: when the timer expires. */
            bypassAltitudeController = true;
            commonThrust             = PRE_TAKEOFF_COMMON_THRUST;
            updatePositionController = false;
            if (getElapsedTime() > PRE_TAKEOFF_DURATION)
                setAutonomousState(TAKEOFF);
            break;

        case TAKEOFF:
            /* Instruction:
                    --- Stage 1: override thrust (blind takeoff thrust), trust
                                 acceleration for position.
                    --- Stage 2: hover at (position, height) = (nextTarget, 1m).
                    --- Stage 3: hover at (position, height) = (nextTarget, 1m).
               Switch to LOITERING: when timer expires. */
            this->referenceHeight = {REFERENCE_HEIGHT};
            if (getElapsedTime() < TAKEOFF_BLIND_DURATION) {
                bypassAltitudeController = true;
                commonThrust =
                    biasManager.getThrustBias() + TAKEOFF_BLIND_MARGINAL_THRUST;
                trustAccelerometerForPosition = true;
            } else if (getElapsedTime() > TAKEOFF_DURATION)
                setAutonomousState(LOITERING);
            break;

        case LOITERING:
            /* Instruction: hover at (position, height) = (nextTarget,
                            referenceHeight).
               Switch to LANDING: if the pilot lowers the throttle enough.
               Switch to CONVERGING: when timer expires. */
            if (getThrottle() <= LANDING_THROTTLE && isLandingEnabled())
                startLanding(true, currentPosition);
            else if (getElapsedTime() > LOITER_DURATION) {
                if (shouldLandAfterLoitering())
                    startLanding(true, currentPosition);
                else if (isNavigatingEnabled())
                    setAutonomousState(CONVERGING);
                /* Stay in LOITERING otherwise. */
            }
            break;

        case CONVERGING:
            /* Instruction: hover at (position, height) = (nextTarget,
                            referenceHeight).
               ! Reset counter if we're no longer within converging distance. !
               Switch to LANDING: if the pilot lowers the throttle enough.
               Switch to NAVIGATING: if QR FSM tells us to.
               Switch to LANDING: if QR FSM tells us to. */
            if (getThrottle() <= LANDING_THROTTLE && isLandingEnabled())
                startLanding(true, currentPosition);
            if (distsq(this->nextTarget, currentPosition) >
                CONVERGENCE_DISTANCE * CONVERGENCE_DISTANCE)
                this->autonomousStateStartTime = getTime();
            break;

        case NAVIGATING:
            /* Instruction: hover at (position, height) = (interpolation point,
                            referenceHeight).
               Switch to LANDING: if the pilot lowers the throttle enough.
               Switch to CONVERGING: when timer expires. */
            if (getThrottle() <= LANDING_THROTTLE && isLandingEnabled())
                startLanding(true, currentPosition);
            if (getElapsedTime() <= this->navigationTime) {
                dx = (nextTarget.x - previousTarget.x) * getElapsedTime() /
                     this->navigationTime;
                dy = (nextTarget.y - previousTarget.y) * getElapsedTime() /
                     this->navigationTime;
                referencePosition =
                    Position(previousTarget.x + dx, previousTarget.y + dy);
            } else {
                setAutonomousState(CONVERGING);
            }
            break;

        case LANDING:
            /* Instruction:
                    --- Stage 1: hover at (position, height) = (nextTarget,
                                 descending referenceHeight).
                    --- Stage 2: override thrust (landing final descent thrust)
                                 and trust accelerometer for position.
                    --- Finished:override thrust (no thrust), don't update
                                 position.
               Switch to IDLE_GROUND: when timer expires. */
            if (this->referenceHeight.z > LANDING_LOWEST_REFERENCE_HEIGHT) {
                this->referenceHeight.z -=
                    LANDING_REFERENCE_HEIGHT_DECREASE_SPEED * SECONDS_PER_TICK;
                /* Reset timer: use it again during the second stage. */
                this->autonomousStateStartTime = getTime();
            } else if (getElapsedTime() < LANDING_BLIND_DURATION) {
                bypassAltitudeController = true;
                commonThrust =
                    biasManager.getThrustBias() + LANDING_BLIND_MARGINAL_THRUST;
                trustAccelerometerForPosition = true;
            } else {
                bypassAltitudeController = true;
                commonThrust             = 0.0;
                updatePositionController = false;
                setAutonomousState(IDLE_GROUND);
            }
            break;

        case WPT: break;

        case ERROR: break;
    }

#pragma endregion

    return AutonomousOutput{bypassAltitudeController,
                            this->referenceHeight,
                            commonThrust,
                            updatePositionController,
                            trustAccelerometerForPosition,
                            referencePosition};
}

void AutonomousController::initAir(Position currentPosition,
                                   AltitudeReference referenceHeight) {
    this->autonomousState          = LOITERING;
    this->autonomousStateStartTime = getTime();
    this->previousTarget           = currentPosition;
    this->nextTarget               = currentPosition;
    this->referenceHeight          = referenceHeight;
    this->qrErrorCount             = 0;
}

void AutonomousController::initGround(Position currentPosition) {
    this->autonomousState          = IDLE_GROUND;
    this->autonomousStateStartTime = getTime();
    this->previousTarget           = currentPosition;
    this->nextTarget               = currentPosition;
    this->qrErrorCount             = 0;
}

AutonomousOutput AutonomousController::update(Position currentPosition) {
    updateQRFSM();
    return updateAutonomousFSM(currentPosition);
}