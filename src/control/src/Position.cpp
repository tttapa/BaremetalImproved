// Original: BareMetal/src/control/navigation.c

#include <Matrix.hpp>
#include <Position.hpp>
#include <TiltCorrection.hpp>

//TODO: implement
void PositionController::checkForNewMeasurement() {}

void PositionController::resetNewMeasurementFlag() {
    PositionController::hasNewMeasurement = false;
}

void PositionController::updateObserver(AttitudeState attitudeState) {

    PositionController::checkForNewMeasurement();
    if (PositionController::hasNewMeasurement) {

        //TODO: komt uit timers -> wat hiermee doen?
        real_t positionMeasurementTimeElapsed =
            getPositionMeasurementTimeElapsed();

        updatePositionObserver(PositionController::stateEstimate,
                               PositionController::measurement, attitudeState,
                               positionMeasurementTimeElapsed);
    }
}

PositionControlSignal PositionController::updateControlSignal() {

    PositionController::checkForNewMeasurement();
    if (PositionController::hasNewMeasurement) {

        //TODO: droneConfiguration & RCTuner
        int currentDroneConfiguration = getDroneConfiguration();
        real_t currentRCTuner         = getRCTuner();

        // Calculate u_k (unclamped)
        getPositionControllerOutput(PositionController::stateEstimate,
                                    PositionController::reference,
                                    PositionController::controlSignal,
                                    PositionController::integralWindup,
                                    currentDroneConfiguration, currentRCTuner);

        // Clamp u_k
        clampPositionControllerOutput(PositionController::controlSignal,
                                      PositionController::integralWindup);
    }

    return PositionController::controlSignal;
}

void PositionController::clampPositionControllerOutput(
    PositionControlSignal controlSignal,
    PositionIntegralWindup integralWindup) {

    if (PositionController::controlSignal.q1ref > PositionController::qRefClamp)
        PositionController::controlSignal.q1ref = PositionController::qRefClamp;
    if (PositionController::controlSignal.q1ref <
        -PositionController::qRefClamp)
        PositionController::controlSignal.q1ref =
            -PositionController::qRefClamp;
    if (PositionController::controlSignal.q2ref > PositionController::qRefClamp)
        PositionController::controlSignal.q2ref = PositionController::qRefClamp;
    if (PositionController::controlSignal.q2ref <
        -PositionController::qRefClamp)
        PositionController::controlSignal.q2ref =
            -PositionController::qRefClamp;
}

void PositionController::initializeController(AttitudeState attitudeState) {

    //TODO: sonar
    ColVector<2> locationMeasurement = getSonarLocationMeasurement();
    real_t heightMeasurement = getSonarHeightMeasurement();

    ColVector<2> correctedPosition = getCorrectedPosition(
        locationMeasurement, heightMeasurement, attitudeState.q);

    // Set reference to middle of the square
    ColVector<2> middleOfSquare;
    middleOfSquare[0] = floorf(correctedPosition[0]) + 0.5;
    middleOfSquare[1] = floorf(correctedPosition[1]) + 0.5;
    PositionController::reference.x =
        middleOfSquare[0] * PositionController::blocksToMeters;
    PositionController::reference.y =
        middleOfSquare[1] * PositionController::blocksToMeters;

    PositionController::stateEstimate.q1 = attitudeState.q[1];
    PositionController::stateEstimate.q2 = attitudeState.q[2];
    PositionController::stateEstimate.x =
        correctedPosition[0] * PositionController::blocksToMeters;
    PositionController::stateEstimate.y =
        correctedPosition[1] * PositionController::blocksToMeters;
    PositionController::stateEstimate.vx = 0;
    PositionController::stateEstimate.vy = 0;
    PositionController::controlSignal    = {};
    PositionController::integralWindup   = {};

    //TODO: blijft dit hier staan?
    // Start loitering mode:
    navigationFSMState = NAV_LOITERING;
    loiteringCounter   = 0;
}