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

void AutonomousController::updateQRFSM() {

    setQRState(readQRState());

    /* Don't update QR codes if the drone is not in CONVERGING state. */
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
            /* Reset error count. */
            AutonomousController::qrErrorCount = 0;

            /* Correct the drone's position if we had to search for the code. */
            correctionX = AutonomousController::nextTarget.x -
                          AutonomousController::nextQRPosition.x;
            correctionY = AutonomousController::nextTarget.y -
                          AutonomousController::nextQRPosition.y;
            if (correctionX != 0.0 && correctionY != 0.0)
                correctDronePosition(correctionX, correctionY);

            /* Set the next target and switch to QR_IDLE. */
            setNextTarget({readQRTargetX(), readQRTargetY()});
            writeQRState(QR_IDLE);
            break;
        case QR_LAND:
            /* Reset error count and switch to QR_IDLE. */
            AutonomousController::qrErrorCount = 0;
            writeQRState(QR_IDLE);
            break;
        case QR_UNKNOWN:
            // TODO: what do we do with unknown QR data?
            /* Reset error count and switch to QR_IDLE. */
            AutonomousController::qrErrorCount = 0;
            writeQRState(QR_IDLE);
            break;
        case QR_ERROR:
            AutonomousController::qrErrorCount++;
            if (AutonomousController::qrErrorCount < getMaxQRErrorCount()) {
                writeQRState(QR_READING); /* Tell IMP to try again. */
            } else {
                AutonomousController::qrErrorCount = 0;
                AutonomousController::qrTilesSearched++;
                PositionReference nextSearchTarget = getNextSearchTarget();
                while (!isValidSearchTarget(nextSearchTarget)) {
                    nextSearchTarget = getNextSearchTarget();
                    AutonomousController::qrTilesSearched++;
                }
                setNextTarget(nextSearchTarget);
                writeQRState(QR_IDLE);
            }
            break;
        default:
            /**
             * In any other case, it's either the Image Processing team's job to
             * continue or it's the Cryptography team's job.
             */
            break;
    }
}

void AutonomousController::initAir(PositionReference currentPosition) {
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
            if (getRCThrottle() > getTakeoffThrottle()) {
                /* Set state to PRE_TAKEOFF and output what it says. */
                setAutonomousState(PRE_TAKEOFF);
                return updateAutonomousFSM(dronePosition);
            } else {
                /* No thrust, don't update position controller. */
                bypassAltitudeController = true;
                commonThrust             = 0.0;
                updatePositionController = false;
            }
            break;

        case IDLE_AIR:
            /* Set state to LOITERING and output what it says. */
            setAutonomousState(LOITERING);
            return updateAutonomousFSM(dronePosition);
            break;

        case PRE_TAKEOFF:
            // TODO: write Enes startup script somewhere else
            // TODO: call it during manual mode
            // TODO: call Enes startup script here too
            /* Set state to TAKEOFF and output what it says. */
            setAutonomousState(TAKEOFF);
            return updateAutonomousFSM(dronePosition);
            break;

        case TAKEOFF:
            if (AutonomousController::elapsedTime < getTakeoffBlindDuration()) {
                /* Takeoff thrust, trust acceleration for position. */
                bypassAltitudeController = true;
                commonThrust =
                    getHoveringThrust() + getTakeoffBlindMarginalThrust();
                trustAccelerometerForPosition = true;
            } else if (AutonomousController::elapsedTime <
                       getTakeoffDuration()) {
                /* Set reference height to default autonomous flight height. */
                AutonomousController::referenceHeight = {getReferenceHeight()};
            } else {
                /* Set state to LOITERING and output what it says. */
                setAutonomousState(LOITERING);
                return updateAutonomousFSM(dronePosition);
            }
            break;

        case LOITERING:
            if (AutonomousController::elapsedTime < getLoiterDuration()) {
                /* Stay at the target location. */
            } else {
                /* Set state to CONVERGING and output what it says. */
                setAutonomousState(CONVERGING);
                return updateAutonomousFSM(dronePosition);
            }
            break;

        case CONVERGING:
            /* Reset counter if we're no longer within converging distance. */
            if (distsq(AutonomousController::nextTarget, dronePosition) >
                getConvergenceDistance() * getConvergenceDistance()) {
                AutonomousController::elapsedTime = 0.0;
            }

            /* Go to NAVIGATING if the Cryptography team sent a new target. */
            if (AutonomousController::qrState == QR_NEW_TARGET) {
                d = dist(AutonomousController::previousTarget,
                         AutonomousController::nextTarget);
                AutonomousController::navigationTime = d / getNavigationSpeed();
                setAutonomousState(NAVIGATING);
                return updateAutonomousFSM(dronePosition);
            }

            /* Stay at the target location otherwise. */
            break;

        case NAVIGATING:
            totalTime = dist(AutonomousController::previousTarget,
                             AutonomousController::nextTarget) /
                        getNavigationSpeed();

            if (AutonomousController::elapsedTime >=
                AutonomousController::navigationTime) {
                /* We've navigated long enough, switch to CONVERGING. */
                setAutonomousState(CONVERGING);
                return updateAutonomousFSM(dronePosition);
            } else {
                /* Interpolate to determine the reference position. */
                factor = AutonomousController::elapsedTime / totalTime;
                dx     = factor * (nextTarget.x - previousTarget.x);
                dy     = factor * (nextTarget.y - previousTarget.y);
                referencePosition = {previousTarget.x + dx,
                                     previousTarget.y + dy};
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
