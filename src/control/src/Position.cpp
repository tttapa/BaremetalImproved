#include <Configuration.hpp>
#include <Globals.hpp>
#include <Position.hpp>
#include <Time.hpp>

/**
 * The largest reference quaternion component that can be sent to the attitude
 * control system is 0.0436.
 */
const real_t REFERENCE_QUATERNION_CLAMP = 0.0436;

real_t dist(Position position1, Position position2) {
    return std::sqrt(distsq(position1, position2));
}

real_t distsq(Position position1, Position position2) {
    real_t dx = position2.x - position1.x;
    real_t dy = position2.y - position1.y;
    return dx * dx + dy * dy;
}

PositionControlSignal
PositionController::clampControlSignal(PositionControlSignal controlSignal) {

    /* Load values from the position controller. */
    real_t q1ref = controlSignal.q1ref;
    real_t q2ref = controlSignal.q2ref;

    /* Clamp q1ref and q2ref. */
    if (q1ref > REFERENCE_QUATERNION_CLAMP)
        q1ref = REFERENCE_QUATERNION_CLAMP;
    if (q1ref < -REFERENCE_QUATERNION_CLAMP)
        q1ref = -REFERENCE_QUATERNION_CLAMP;
    if (q2ref > REFERENCE_QUATERNION_CLAMP)
        q2ref = REFERENCE_QUATERNION_CLAMP;
    if (q2ref < -REFERENCE_QUATERNION_CLAMP)
        q2ref = -REFERENCE_QUATERNION_CLAMP;

    return PositionControlSignal{q1ref, q2ref};
}

void PositionController::correctPosition(real_t correctionX,
                                         real_t correctionY) {
    PositionController::stateEstimate.x += correctionX;
    PositionController::stateEstimate.y += correctionY;
}

void PositionController::init() {
    /* Reset the position controller. */
    PositionController::stateEstimate       = {};
    PositionController::integralWindup      = {};
    PositionController::controlSignal       = {};
    PositionController::lastMeasurementTime = 0.0;
}

PositionControlSignal
PositionController::updateControlSignal(PositionReference reference) {

    /* Calculate integral windup. */
    PositionController::integralWindup =
        codegenIntegralWindup(PositionController::integralWindup, reference);

    /* Calculate control signal (unclamped). */
    PositionController::controlSignal = codegenControlSignal(
        PositionController::stateEstimate, reference,
        PositionController::integralWindup, getDroneConfiguration());

    /* Clamp control signal. */
    PositionController::controlSignal =
        clampControlSignal(PositionController::controlSignal);

    return PositionController::controlSignal;
}

void PositionController::updateObserver(Quaternion orientation,
                                        PositionMeasurement measurement) {
    /* Calculate the current state estimate. */
    PositionController::stateEstimate = codegenCurrentStateEstimate(
        PositionController::stateEstimate, measurement, orientation,
        getDroneConfiguration());
}