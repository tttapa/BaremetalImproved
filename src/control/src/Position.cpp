#include <MiscInstances.hpp>
#include <Position.hpp>
#include <Time.hpp>

/**
 * The largest reference quaternion component that can be sent to the attitude
 * control system is 0.0436.
 */
static constexpr real_t REFERENCE_QUATERNION_CLAMP = 0.0436;

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
    this->stateEstimate.x += correctionX;
    this->stateEstimate.y += correctionY;
}

void PositionController::init() {

    /* Reset the position controller. */
    this->stateEstimate       = {};
    this->integralWindup      = {};
    this->controlSignal       = {};
    this->lastMeasurementTime = 0.0;
}

PositionControlSignal
PositionController::updateControlSignal(PositionReference reference) {

    /* Save the reference position. */
    this->reference = reference;

    /* Calculate integral windup. */
    this->integralWindup = codegenIntegralWindup(
        this->integralWindup, reference, this->stateEstimate,
        configManager.getControllerConfiguration());

    /* Calculate control signal (unclamped). */
    this->controlSignal = codegenControlSignal(
        this->stateEstimate, reference, this->integralWindup,
        configManager.getControllerConfiguration());

    /* Clamp control signal. */
    this->controlSignal = clampControlSignal(this->controlSignal);

    return this->controlSignal;
}

void PositionController::updateObserver(Quaternion orientation,
                                        real_t currentTime,
                                        PositionMeasurement measurement) {
    /* Calculate the current state estimate. */
    this->stateEstimate = codegenCurrentStateEstimate(
        this->stateEstimate, measurement, orientation,
        currentTime - lastMeasurementTime,
        configManager.getControllerConfiguration());

    /* Store the measurement time. */
    this->lastMeasurementTime = currentTime;
}

void PositionController::updateObserverBlind(Quaternion orientation) {

    PositionStateBlind stateBlind = {
        this->stateEstimate.x,
        this->stateEstimate.y,
        this->stateEstimate.vx,
        this->stateEstimate.vy,
    };
    PositionControlSignalBlind controlSignalBlind = {orientation[1],
                                                     orientation[2]};

    this->stateEstimate =
        codegenCurrentStateEstimateBlind(stateBlind, controlSignalBlind);
}
