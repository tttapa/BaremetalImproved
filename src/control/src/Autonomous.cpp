#include <Autonomous.hpp>
#include <Globals.hpp>
#include <SoftwareConstants.hpp>
#include <Time.hpp>

/* Use software constants from the AUTONOMOUS namespace. */
using namespace AUTONOMOUS;

bool isValidSearchTarget(PositionReference position) {
    return position.x >= POSITION::getXMin() &&
           position.x <= POSITION::getXMax() &&
           position.y >= POSITION::getYMin() &&
           position.y <= POSITION::getYMax();
}

real_t AutonomousController::getElapsedTime() {
    return getTime() - AutonomousController::autonomousStateStartTime;
}

PositionReference AutonomousController::getNextSearchTarget() {
    real_t x = AutonomousController::nextQRPosition.x;
    real_t y = AutonomousController::nextQRPosition.y;

    /* Spiral outward until we reach the next tile to check. */
    real_t dx          = 1.0;
    real_t dy          = 0.0;
    int tilesUntilTurn = 0;
    int nextTurnIndex  = 1;
    for (int i = 0; i < AutonomousController::qrTilesSearched; i++) {
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
    return PositionReference{x, y};
}

void AutonomousController::setAutonomousState(AutonomousState state) {
    AutonomousController::autonomousState          = state;
    AutonomousController::autonomousStateStartTime = getTime();
}

void AutonomousController::setNextTarget(PositionReference target) {
    AutonomousController::previousTarget = AutonomousController::nextTarget;
    AutonomousController::nextTarget     = target;
}

void AutonomousController::setQRState(int state) {
    switch (state) {
        case QR_IDLE: AutonomousController::qrState = QR_IDLE; return;
        case QR_READING: AutonomousController::qrState = QR_READING; return;
        case QR_CRYPTO_BUSY:
            AutonomousController::qrState = QR_CRYPTO_BUSY;
            return;
        case QR_NEW_TARGET:
            AutonomousController::qrState = QR_NEW_TARGET;
            return;
        case QR_LAND: AutonomousController::qrState = QR_LAND; return;
        case QR_UNKNOWN: AutonomousController::qrState = QR_UNKNOWN; return;
        case QR_ERROR: AutonomousController::qrState = QR_ERROR; return;
        default: AutonomousController::qrState = QR_IDLE;
    }
}

void AutonomousController::startLanding(bool landAtCurrentPosition,
                                        PositionReference currentPosition) {
    if (landAtCurrentPosition)
        setNextTarget(currentPosition);

    // TODO: landing script init
    setAutonomousState(LANDING);
}

void AutonomousController::startNavigating(PositionReference nextQRPosition) {
    setNextTarget(nextQRPosition);
    real_t d = dist(AutonomousController::previousTarget,
                    AutonomousController::nextTarget);
    AutonomousController::navigationTime = d / getNavigationSpeed();
    setAutonomousState(NAVIGATING);
}

void AutonomousController::updateQRFSM() {

    /* Load the QR state from shared memory. */
    setQRState(readQRState());

    /* Don't update QR FSM if the drone is not in CONVERGING state. */
    if (AutonomousController::autonomousState != CONVERGING)
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
    switch (AutonomousController::qrState) {
        case QR_IDLE:
            /* Let the Image Processing team take a picture if we have converged
               on our target. */
            if (getElapsedTime() > getConvergenceDuration())
                writeQRState(QR_READING);
            break;
        case QR_NEW_TARGET:
            /* Reset error count and search count. */
            AutonomousController::qrErrorCount    = 0;
            AutonomousController::qrTilesSearched = 0;

            /* Correct the drone's position if the drone got lost, and we had to
               search for the code. */
            correctionX = AutonomousController::nextTarget.x -
                          AutonomousController::nextQRPosition.x;
            correctionY = AutonomousController::nextTarget.y -
                          AutonomousController::nextQRPosition.y;
            if (correctionX != 0.0 && correctionY != 0.0)
                correctDronePosition(correctionX, correctionY);

            /* Tell the autonomous controller's FSM to start navigating to the
               position of the next QR code sent by the Cryptography team. */
            /* Switch this FSM to QR_IDLE. */
            startNavigating({readQRTargetX(), readQRTargetY()});
            writeQRState(QR_IDLE);
            break;
        case QR_LAND:
            /* Reset error count and search count. */
            AutonomousController::qrErrorCount    = 0;
            AutonomousController::qrTilesSearched = 0;

            /* Tell the autonomous controller's FSM to start landing. */
            /* Switch this FSM to QR_IDLE. */
            startLanding(false, {});
            writeQRState(QR_IDLE);
            break;
        case QR_UNKNOWN:
            /* Reset error count and search count. */
            AutonomousController::qrErrorCount    = 0;
            AutonomousController::qrTilesSearched = 0;
            // TODO: what do we do with unknown QR data?

            /* Switch this FSM to QR_IDLE. */
            writeQRState(QR_IDLE);
            break;
        case QR_ERROR:
            AutonomousController::qrErrorCount++;
            if (AutonomousController::qrErrorCount <= getMaxQRErrorCount()) {
                writeQRState(QR_READING); /* Tell IMP to try again. */
            } else {

                /* Start (or continue) spiral-searching for QR code. */
                AutonomousController::qrErrorCount = 0;
                AutonomousController::qrTilesSearched++;
                PositionReference nextSearchTarget = getNextSearchTarget();
                while (!isValidSearchTarget(nextSearchTarget) &&
                       AutonomousController::qrTilesSearched <
                           getMaxQRSearchCount()) {
                    nextSearchTarget = getNextSearchTarget();
                    AutonomousController::qrTilesSearched++;
                }

                /* Valid search target, so set it as the next target. */
                /* Switch this FSM to QR_IDLE. */
                if (AutonomousController::qrTilesSearched <
                    getMaxQRSearchCount()) {
                    setNextTarget(nextSearchTarget);
                    writeQRState(QR_IDLE);
                }

                /* We've run out of tiles to search, so have the drone land. */
                /* Switch this FSM to QR_IDLE. */
                else {
                    startLanding(false, {});
                    writeQRState(QR_IDLE);
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
AutonomousController::updateAutonomousFSM(PositionReference currentPosition) {

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
    bool bypassAltitudeController       = false;
    real_t commonThrust                 = 0.0;
    bool updatePositionController       = true;
    bool trustAccelerometerForPosition  = false;
    PositionReference referencePosition = AutonomousController::nextTarget;

    /* Implement FSM logic. */
    real_t dx, dy;
    switch (AutonomousController::autonomousState) {

        case IDLE_GROUND:
            /* Instruction: override thrust (no thrust), don't update position
               controller. */
            bypassAltitudeController = true;
            commonThrust             = 0.0;
            updatePositionController = false;

            /* Switch to PRE_TAKEOFF when the throttle is raised high enough. */
            if (getRCThrottle() > getTakeoffThrottle())
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
            AutonomousController::referenceHeight = {getReferenceHeight()};

            /* Takeoff stage 1... Instruction = override thrust (blind takeoff
               thrust), trust acceleration for position. */
            if (getElapsedTime() < getTakeoffBlindDuration()) {
                bypassAltitudeController = true;
                commonThrust =
                    getHoveringThrust() + getTakeoffBlindMarginalThrust();
                trustAccelerometerForPosition = true;
            }
            /* Takeoff stage 2... Instruction = hover at (position, height) = 
               (nextTarget, 1 meter). */

            /* Takeoff finished... Instruction = hover at (position, height) =
               (nextTarget, 1 meter). */
            /* Switch to LOITERING. */
            else if (getElapsedTime() > getTakeoffDuration())
                setAutonomousState(LOITERING);
            break;

        case LOITERING:
            /* Switch to LANDING if the pilot lowers the throttle enough. */
            if (getRCThrottle() <= getLandingThrottle())
                startLanding(true, currentPosition);

            /* Instruction = hover at (position, height) = (nextTarget,
               referenceHeight). */
            /* Switch to CONVERGING if we've loitered long enough. */
            if (getElapsedTime() > getLoiterDuration())
                setAutonomousState(CONVERGING);
            break;

        case CONVERGING:
            /* Switch to LANDING if the pilot lowers the throttle enough. */
            if (getRCThrottle() <= getLandingThrottle())
                startLanding(true, currentPosition);

            /* Reset counter if we're no longer within converging distance. */
            if (distsq(AutonomousController::nextTarget, currentPosition) >
                getConvergenceDistance() * getConvergenceDistance()) {
                AutonomousController::autonomousStateStartTime = getTime();
            }
            /* Instruction = hover at (position, height) = (nextTarget,
               referenceHeight). */
            /* Stay in this state until the QR FSM moves us to NAVIGATING or
               LANDING. */
            break;

        case NAVIGATING:
            /* Switch to LANDING if the pilot lowers the throttle enough. */
            if (getRCThrottle() <= getLandingThrottle())
                startLanding(true, currentPosition);

            /* Navigating... Instruction = hover at (position, height) =
               (interpolation point, referenceHeight). */
            if (getElapsedTime() <= AutonomousController::navigationTime) {
                dx = (nextTarget.x - previousTarget.x) * getElapsedTime() /
                     AutonomousController::navigationTime;
                dy = (nextTarget.y - previousTarget.y) * getElapsedTime() /
                     AutonomousController::navigationTime;
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
            if (AutonomousController::referenceHeight.z >
                getLandingLowestReferenceHeight()) {
                AutonomousController::referenceHeight.z -=
                    getLandingReferenceHeightDecreaseSpeed() *
                    getSecondsPerTick();

                /* Reset autonomous state timer, so we can use it during the
                   second stage. */
                AutonomousController::autonomousStateStartTime = getTime();
            }

            /* Landing stage 2... Instruction = override thrust (landing final
               descent thrust) and trust accelerometer for position. */
            else if (getElapsedTime() < getLandingBlindDuration()) {
                bypassAltitudeController = true;
                commonThrust =
                    getHoveringThrust() + getLandingBlindMarginalThrust();
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

    return AutonomousOutput{
        bypassAltitudeController,
        AutonomousController::referenceHeight,
        commonThrust,
        updatePositionController,
        trustAccelerometerForPosition,
        referencePosition,
    };
}

void AutonomousController::initAir(PositionReference currentPosition,
                                   AltitudeReference referenceHeight) {
    AutonomousController::autonomousState          = IDLE_AIR;
    AutonomousController::autonomousStateStartTime = getTime();
    AutonomousController::previousTarget           = currentPosition;
    AutonomousController::nextTarget               = currentPosition;
    AutonomousController::referenceHeight          = referenceHeight;
    AutonomousController::qrErrorCount             = 0;
}

void AutonomousController::initGround(PositionReference currentPosition) {
    AutonomousController::autonomousState          = IDLE_GROUND;
    AutonomousController::autonomousStateStartTime = getTime();
    AutonomousController::previousTarget           = currentPosition;
    AutonomousController::nextTarget               = currentPosition;
    AutonomousController::qrErrorCount             = 0;
}

AutonomousOutput
AutonomousController::update(PositionReference currentPosition) {
    updateQRFSM();
    return updateAutonomousFSM(currentPosition);
}