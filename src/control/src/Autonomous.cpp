#include <Autonomous.hpp>
#include <ControllerInstances.hpp>
#include <Globals.hpp>
#include <InputBias/InputBias.hpp>
#include <Position.hpp>
#include <Time.hpp>

/**
 * If the drone is stays with 0.10 meters of its destination for a period of
 * time, then it will have converged on its target.
 */
const real_t CONVERGENCE_DISTANCE = 0.10;

/**
 * If the drone is stays with a certain distance of its destination for 1
 * second, then it will have converged on its target.
 */
const real_t CONVERGENCE_DURATION = 1.0;

/**
 * The blind stage of the landing, meaning the sonar is not accurate anymore,
 * lasts 2.5 seconds.
 */
const real_t LANDING_BLIND_DURATION = 2.5;

/**
 * During the blind stage of the landing, meaning the sonar is not accurate
 * anymore, a marginal signal of 1% below the hovering signal will be sent to
 * the "common motor".
 */
const real_t LANDING_BLIND_MARGINAL_THRUST = -0.01;

/**
 * In the first stage of the landing procedure, when the sonar is accurate, the
 * reference height will decrease until it hits 0.25 meters.
 */
const real_t LANDING_LOWEST_REFERENCE_HEIGHT = 0.25;
;

/**
 * In the first stage of the landing procedure, when the sonar is accurate, the
 * reference height will decrease at a speed of 0.25 m/s until it hits the
 * minimum value, see getLandingLowestReferenceHeight().
 */
const real_t LANDING_REFERENCE_HEIGHT_DECREASE_SPEED = 0.25;

/**
 * If the autonomous controller is in the state LOITERING, NAVIGATING or
 * CONVERGING, then the drone will land if the throttle value goes below
 * 0.05.
 */
const real_t LANDING_THROTTLE = 0.05;

/**
 * The autonomous controller will loiter for 15 seconds before navigating.
 */
const real_t LOITER_DURATION = 15.0;

/**
 * If the Cryptography team fails to decrypt the image sent by the Image
 * Processing team 3 times in a row, then it's likely that the drone is not
 * directly above the QR code. Therefore, it will start searching for it.
 */
const int MAX_QR_ERROR_COUNT = 3;

// TODO: either 25 for radius 2 or 49 for radius 3
/**
 * If the Cryptography team fails to decrypt the image sent by the Image
 * Processing team too many times, then it's likely that the drone is not 
 * directly above the QR code. Therefore, it will start searching for it. After
 * 25 failed tiles (radius 2 around proposed QR position), the drone will stop
 * searching and attempt to land.
 */
const int MAX_QR_SEARCH_COUNT = 25;

/**
 * When the drone is navigating in autonomous mode, the reference will travel at
 * a speed of 0.5 m/s.
 */
const real_t NAVIGATION_SPEED = 0.5;

/** The normal reference height for the drone in autonomous mode is 1 meter. */
const real_t REFERENCE_HEIGHT = 1.0;

/**
 * The blind stage of the takeoff, meaning the sonar is not yet accurate,
 * lasts 0.5 seconds.
 */
const real_t TAKEOFF_BLIND_DURATION = 0.5;

/**
 * During the blind stage of the takeoff, meaning the sonar is not yet accurate,
 * a marginal signal of 3% above the hovering signal will be sent to the "common
 * motor".
 */
const real_t TAKEOFF_BLIND_MARGINAL_THRUST = 0.03;

/** The entire takeoff will last 2 seconds. */
const real_t TAKEOFF_DURATION = 2.0;

/**
 * If the autonomous controller is in the state IDLE_GROUND, then the drone
 * will take off if the throttle value exceeds 0.50.
 */
const real_t TAKEOFF_THROTTLE = 0.50;

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

void AutonomousController::setQRState(QRFSMState nextState) {
    this->qrState = nextState;
}

void AutonomousController::startLanding(bool shouldLandAtCurrentPosition,
                                        Position currentPosition) {
    if (shouldLandAtCurrentPosition)
        setNextTarget(currentPosition);

    // TODO: landing script init
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
    setQRState(readQRState());

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
     */

    /* Implement FSM logic. */
    real_t correctionX, correctionY;
    switch (this->qrState) {
        case QRFSMState::IDLE:
            /* Let the Image Processing team take a picture if we have converged
               on our target. */
            if (getElapsedTime() > CONVERGENCE_DURATION)
                writeQRState(QRFSMState::QR_READ_REQUEST);
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
                correctDronePosition(correctionX, correctionY);

            /* Tell the autonomous controller's FSM to start navigating to the
               position of the next QR code sent by the Cryptography team. */
            /* Switch this FSM to QR_IDLE. */
            startNavigating({readQRTargetX(), readQRTargetY()});
            writeQRState(QRFSMState::IDLE);
            break;
        case QRFSMState::LAND:
            /* Reset error count and search count. */
            this->qrErrorCount    = 0;
            this->qrTilesSearched = 0;

            /* Tell the autonomous controller's FSM to start landing. */
            /* Switch this FSM to QR_IDLE. */
            startLanding(false, {});
            writeQRState(QRFSMState::IDLE);
            break;
        case QRFSMState::QR_UNKNOWN:
            /* Reset error count and search count. */
            this->qrErrorCount    = 0;
            this->qrTilesSearched = 0;
            // TODO: what do we do with unknown QR data?

            /* Switch this FSM to QR_IDLE. */
            writeQRState(QRFSMState::IDLE);
            break;
        case QRFSMState::ERROR:
            this->qrErrorCount++;
            if (this->qrErrorCount <= MAX_QR_ERROR_COUNT) {
                /* Tell IMP to try again. */
                writeQRState(QRFSMState::QR_READ_REQUEST);
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
                    writeQRState(QRFSMState::IDLE);
                }

                /* We've run out of tiles to search, so have the drone land. */
                /* Switch this FSM to QR_IDLE. */
                else {
                    startLanding(false, {});
                    writeQRState(QRFSMState::IDLE);
                }
            }
            break;
        default:
            /* In any other case, it's not our job to update the QR FSM. It's up
               to either the Image Processing team or the Cryptography team. */
            break;
    }
}

AutonomousOutput
AutonomousController::updateAutonomousFSM(Position currentPosition) {

    /**
     * ============================ AUTONOMOUS FSM ============================
     * -------------- START ------------- | -------------- ERROR --------------
     * Idle_Ground: if drone is grounded  | Takeoff Stage 2 -> Error
     * Idle_Air: if drone is airborne     | Loitering -> Error
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
     * Idle_Air -> Loitering              |
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
               controller. */
            bypassAltitudeController = true;
            commonThrust             = 0.0;
            updatePositionController = false;

            /* Switch to PRE_TAKEOFF when the throttle is raised high enough. */
            if (getRCThrottle() > TAKEOFF_THROTTLE)
                setAutonomousState(PRE_TAKEOFF);
            break;

        case IDLE_AIR:
            /* Instruction: hover at (position, height) = (nextTarget,
               referenceHeight) which are both set in initAir(). */
            /* Switch to LOITERING. */
            setAutonomousState(LOITERING);
            break;

        case PRE_TAKEOFF:
            /* Instruction: // TODO: startup script */
            // TODO: write Enes startup script somewhere else
            // TODO: call it during manual mode
            // TODO: call Enes startup script here too

            /* Switch to TAKEOFF when PRE_TAKEOFF script has finished. */
            setAutonomousState(TAKEOFF);
            break;

        case TAKEOFF:
            /* Set reference height to 1 meter. */
            this->referenceHeight = {REFERENCE_HEIGHT};

            /* Takeoff stage 1... Instruction = override thrust (blind takeoff
               thrust), trust acceleration for position. */
            if (getElapsedTime() < TAKEOFF_BLIND_DURATION) {
                bypassAltitudeController = true;
                commonThrust =
                    inputBias.getThrustBias() + TAKEOFF_BLIND_MARGINAL_THRUST;
                trustAccelerometerForPosition = true;
            }
            /* Takeoff stage 2... Instruction = hover at (position, height) = 
               (nextTarget, 1 meter). */

            /* Takeoff finished... Instruction = hover at (position, height) =
               (nextTarget, 1 meter). */
            /* Switch to LOITERING. */
            else if (getElapsedTime() > TAKEOFF_DURATION)
                setAutonomousState(LOITERING);
            break;

        case LOITERING:
            /* Switch to LANDING if the pilot lowers the throttle enough. */
            if (getRCThrottle() <= LANDING_THROTTLE)
                startLanding(true, currentPosition);

            /* Instruction = hover at (position, height) = (nextTarget,
               referenceHeight). */
            /* Switch to CONVERGING if we've loitered long enough. */
            if (getElapsedTime() > LOITER_DURATION)
                setAutonomousState(CONVERGING);
            break;

        case CONVERGING:
            /* Switch to LANDING if the pilot lowers the throttle enough. */
            if (getRCThrottle() <= LANDING_THROTTLE)
                startLanding(true, currentPosition);

            /* Reset counter if we're no longer within converging distance. */
            if (distsq(this->nextTarget, currentPosition) >
                CONVERGENCE_DISTANCE * CONVERGENCE_DISTANCE) {
                this->autonomousStateStartTime = getTime();
            }
            /* Instruction = hover at (position, height) = (nextTarget,
               referenceHeight). */
            /* Stay in this state until the QR FSM moves us to NAVIGATING or
               LANDING. */
            break;

        case NAVIGATING:
            /* Switch to LANDING if the pilot lowers the throttle enough. */
            if (getRCThrottle() <= LANDING_THROTTLE)
                startLanding(true, currentPosition);

            /* Navigating... Instruction = hover at (position, height) =
               (interpolation point, referenceHeight). */
            if (getElapsedTime() <= this->navigationTime) {
                dx = (nextTarget.x - previousTarget.x) * getElapsedTime() /
                     this->navigationTime;
                dy = (nextTarget.y - previousTarget.y) * getElapsedTime() /
                     this->navigationTime;
                referencePosition = {previousTarget.x + dx,
                                     previousTarget.y + dy};
            }

            /* Finished navigating... Instruction = hover at (position, height)
               = (nextTarget, referenceHeight). */
            /* Switch to CONVERGING. */
            else
                setAutonomousState(CONVERGING);
            break;

        case LANDING:
            /* Landing stage 1... Instruction = hover at (position, height) =
               (nextTarget, referenceHeight). nextTarget is set when the method
               startNavigating() is called; referenceHeight is updated in this
               block of code. */
            if (this->referenceHeight.z > LANDING_LOWEST_REFERENCE_HEIGHT) {
                this->referenceHeight.z -=
                    LANDING_REFERENCE_HEIGHT_DECREASE_SPEED * SECONDS_PER_TICK;

                /* Reset autonomous state timer, so we can use it during the
                   second stage. */
                this->autonomousStateStartTime = getTime();
            }

            /* Landing stage 2... Instruction = override thrust (landing final
               descent thrust) and trust accelerometer for position. */
            else if (getElapsedTime() < LANDING_BLIND_DURATION) {
                bypassAltitudeController = true;
                commonThrust =
                    inputBias.getThrustBias() + LANDING_BLIND_MARGINAL_THRUST;
                trustAccelerometerForPosition = true;
            }

            /* Landing finished... Instruction = override thrust (no thrust),
               dont' update position controller. */
            /* Switch to IDLE_GROUND. */
            else {
                bypassAltitudeController = true;
                commonThrust             = 0.0;
                updatePositionController = false;
                setAutonomousState(IDLE_GROUND);
            }
            break;

        case WPT: break;

        case ERROR: break;
    }

    return AutonomousOutput{bypassAltitudeController,
                            this->referenceHeight,
                            commonThrust,
                            updatePositionController,
                            trustAccelerometerForPosition,
                            referencePosition};
}

void AutonomousController::initAir(Position currentPosition,
                                   AltitudeReference referenceHeight) {
    this->autonomousState          = IDLE_AIR;
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