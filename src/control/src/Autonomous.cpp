#include <Autonomous.hpp>
#include <Globals.hpp>
#include <SoftwareConstants.hpp>

/* Use software constants from the AUTONOMOUS namespace. */
using namespace AUTONOMOUS;

PositionReference AutonomousController::getNextSearchTarget() {

    /* Spiral outwards until we reach the next tile to check. */
    real_t x = AutonomousController::nextQRPosition.x;
    real_t y = AutonomousController::nextQRPosition.y;

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

bool isValidSearchTarget(PositionReference position) {
    return position.x >= POSITION::getXMin() &&
           position.x <= POSITION::getXMax() &&
           position.y >= POSITION::getYMin() &&
           position.y >= POSITION::getYMax();
}

void AutonomousController::setAutonomousState(AutonomousState nextState) {
    AutonomousController::autonomousState = nextState;
    AutonomousController::elapsedTime     = 0.0;
}

void AutonomousController::setNextTarget(PositionReference target) {
    AutonomousController::previousTarget = AutonomousController::nextTarget;
    AutonomousController::nextTarget     = target;
}

void AutonomousController::setQRState(int nextState) {
    switch (nextState) {
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

void AutonomousController::startLanding() {
    // TODO: landing script init
    setAutonomousState(LANDING);
}

void AutonomousController::startNavigating() {
    real_t d = dist(AutonomousController::previousTarget,
                    AutonomousController::nextTarget);
    AutonomousController::navigationTime = d / getNavigationSpeed();
    setAutonomousState(NAVIGATING);
}

void AutonomousController::updateQRFSM() {

    setQRState(readQRState());

    /* Don't update QR FSM if the drone is not in CONVERGING state. */
    if (AutonomousController::autonomousState != CONVERGING)
        return;

    /**
     *  Implement FSM logic.
     * 
     * ============ QR FSM ============
     * ------------- START ------------
     * QR_Idle
     * 
     * ---------- MAIN CYCLE ----------
     * QR_Idle -> QR_Reading
     * QR_Reading -> QR_Crypto_Busy
     * 
     * QR_Crypto_Busy -> QR_Error
     * QR_Crypto_Busy -> QR_New_Target
     * QR_Crypto_Busy -> QR_Land
     * QR_Crypto_Busy -> QR_Unknown
     * 
     * QR_Error -> QR_Reading
     * QR_New_Target -> QR_Idle
     * QR_Land -> QR_Idle
     * QR_Unknown -> QR_Idle
     */
    real_t correctionX, correctionY;
    switch (AutonomousController::qrState) {
        case QR_IDLE:
            /* Let the Image Processing team take a picture if convergence. */
            if (AutonomousController::elapsedTime > getConvergenceDuration())
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

            /* Set the next target, tell the autonomous controller's FSM to
               start navigating, and switch this FSM to QR_IDLE. */
            setNextTarget(PositionReference{readQRTargetX(), readQRTargetY()});
            startNavigating();
            writeQRState(QR_IDLE);
            break;
        case QR_LAND:
            /* Reset error count and search count. Switch to QR_IDLE. */
            AutonomousController::qrErrorCount    = 0;
            AutonomousController::qrTilesSearched = 0;

            /* Tell the autonomous controller's FSM to start landing, and switch
               this FSM to QR_IDLE. */
            startLanding();
            writeQRState(QR_IDLE);
            break;
        case QR_UNKNOWN:
            /* Reset error count and search count. Switch to QR_IDLE */
            AutonomousController::qrErrorCount    = 0;
            AutonomousController::qrTilesSearched = 0;
            // TODO: what do we do with unknown QR data?
            writeQRState(QR_IDLE);
            break;
        case QR_ERROR:
            AutonomousController::qrErrorCount++;
            if (AutonomousController::qrErrorCount <= getMaxQRErrorCount()) {
                writeQRState(QR_READING); /* Tell IMP to try again. */
            } else {

                /* Spiral-searching for QR code. */
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
                if (AutonomousController::qrTilesSearched <
                    getMaxQRSearchCount()) {
                    setNextTarget(nextSearchTarget);
                    writeQRState(QR_IDLE);
                }

                /* We've run out of tiles to search, so have the drone land. */
                else {
                    startLanding();
                }
            }
            break;
        default:
            /* In any other case, it's either the Image Processing team's job to
               continue or it's the Cryptography team's job. */
            break;
    }
}

void AutonomousController::initAir(PositionReference currentPosition,
                                   AltitudeReference currentHeight) {
    AutonomousController::autonomousState = IDLE_AIR;
    AutonomousController::elapsedTime     = 0.0;
    AutonomousController::previousTarget  = currentPosition;
    AutonomousController::nextTarget      = currentPosition;
    AutonomousController::qrErrorCount    = 0;
}

void AutonomousController::initGround(PositionReference currentPosition) {
    AutonomousController::autonomousState = IDLE_GROUND;
    AutonomousController::elapsedTime     = 0.0;
    AutonomousController::previousTarget  = currentPosition;
    AutonomousController::nextTarget      = currentPosition;
    AutonomousController::qrErrorCount    = 0;
}

AutonomousOutput
AutonomousController::updateAutonomousFSM(PositionReference dronePosition) {

    /**
     * ======== AUTONOMOUS FSM ========
     * ------------- START ------------
     * Idle_Ground: if drone is grounded
     * Idle_Air: if drone is airborne
     * 
     * ---------- MAIN CYCLE ----------
     * Idle_Ground -> Pre_Takeoff
     * Pre_Takeoff -> Takeoff
     * Takeoff -> Loitering
     * Loitering -> Converging
     * Converging <-> Navigating
     * Converging -> Landing
     * Landing -> Idle
     * 
     * ------- MID-FLIGHT CYCLE -------
     * Idle_Air -> Loitering
     * Loitering -> Converging
     * Converging <-> Navigating
     * Converging -> Landing
     * Landing -> Idle
     * 
     * ------------ ERROR ------------
     * Takeoff Stage 2 -> Error
     * Loitering -> Error
     * Converging -> Error
     * Navigating -> Error
     * Landing Stage 1 -> Error
     * 
     * --- WIRELESS POWER TRANSFER ---
     * Idle_Ground -> WPT
     * WPT -> Idle_Ground
     */

    /* Default values for the autonomous controller's output. */
    bool bypassAltitudeController       = false;
    real_t commonThrust                 = 0.0;
    bool updatePositionController       = true;
    bool trustAccelerometerForPosition  = false;
    PositionReference referencePosition = AutonomousController::nextTarget;

    real_t d, totalTime, factor, dx, dy;
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
            /* Instruction: hover at position=nextTarget, which should be the
               current position, and height= & reference height. */
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
            /* Set reference height to 1m. */
            AutonomousController::referenceHeight = {getReferenceHeight()};

            /* Takeoff stage 1... Instruction = override thrust (blind takeoff
               thrust), trust acceleration for position. */
            if (AutonomousController::elapsedTime < getTakeoffBlindDuration()) {
                bypassAltitudeController = true;
                commonThrust =
                    getHoveringThrust() + getTakeoffBlindMarginalThrust();
                trustAccelerometerForPosition = true;
            }
            /* Takeoff stage 2... Instruction = hover at current position &
               reference height. */

            /* Takeoff finished... Instruction = hover at current position &
               reference height. */
            /* Switch to LOITERING. */
            else if (AutonomousController::elapsedTime > getTakeoffDuration())
                setAutonomousState(LOITERING);
            break;

        case LOITERING:
            /* Instruction = hover at current position & reference height. */
            /* Switch to CONVERGING if we've loitered long enough. */
            if (AutonomousController::elapsedTime > getLoiterDuration())
                setAutonomousState(CONVERGING);
            break;

        case CONVERGING:
            /* Reset counter if we're no longer within converging distance. */
            if (distsq(AutonomousController::nextTarget, dronePosition) >
                getConvergenceDistance() * getConvergenceDistance()) {
                AutonomousController::elapsedTime = 0.0;
            }
            /* Instruction = hover at current position & reference height. */
            /* Stay in this state until the QR FSM moves us to NAVIGATING or
               LANDING. */
            break;

        case NAVIGATING:
            /* Navigating... Instruction = hover at interpolation point &
               current reference height. */
            if (AutonomousController::elapsedTime <=
                AutonomousController::navigationTime) {
                /* Interpolate to determine the reference position. */
                factor = AutonomousController::elapsedTime / totalTime;
                dx     = factor * (nextTarget.x - previousTarget.x);
                dy     = factor * (nextTarget.y - previousTarget.y);
                referencePosition = {previousTarget.x + dx,
                                     previousTarget.y + dy};
            }

            /* Finished navigating... Instruction = hover at target 
            else {
                                /* We've navigated long enough, switch to CONVERGING. */
            setAutonomousState(CONVERGING);
            return updateAutonomousFSM(dronePosition);
    }
    else {
    }
    break;

    case LANDING: break;

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
