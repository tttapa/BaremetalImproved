#include <Position.hpp>

/* Includes from src. */
#include <MiscInstances.hpp>
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

void PositionController::clampControlSignal() {
    if (this->controlSignal.q1ref > REFERENCE_QUATERNION_CLAMP)
        this->controlSignal.q1ref = REFERENCE_QUATERNION_CLAMP;
    if (this->controlSignal.q1ref < -REFERENCE_QUATERNION_CLAMP)
        this->controlSignal.q1ref = -REFERENCE_QUATERNION_CLAMP;
    if (this->controlSignal.q2ref > REFERENCE_QUATERNION_CLAMP)
        this->controlSignal.q2ref = REFERENCE_QUATERNION_CLAMP;
    if (this->controlSignal.q2ref < -REFERENCE_QUATERNION_CLAMP)
        this->controlSignal.q2ref = -REFERENCE_QUATERNION_CLAMP;
}

void PositionController::init(Position currentPosition) {

    /* Reset the position controller. */
    this->stateEstimate       = {0.0, 0.0, currentPosition, 0.0, 0.0};
    this->integralWindup      = {};
    this->controlSignal       = {};
    this->lastMeasurementTime = getTime();
}

void PositionController::correctPositionEstimate(Position correctPosition) {
    // TODO: meters2blocks
    const real_t BLOCKS2METERS = 0.30;
    const real_t METERS2BLOCKS = 1.0 / BLOCKS2METERS;
    real_t dx                  = correctPosition.x - stateEstimate.p.x;
    real_t dy                  = correctPosition.y - stateEstimate.p.y;
    real_t offsetXBlocks       = round(dx * METERS2BLOCKS);
    real_t offsetYBlocks       = round(dy * METERS2BLOCKS);
    this->stateEstimate.p.x += offsetXBlocks * BLOCKS2METERS;
    this->stateEstimate.p.y += offsetYBlocks * BLOCKS2METERS;
}

PositionControlSignal
PositionController::updateControlSignal(PositionReference reference) {

    /* Save the reference position. */
    this->reference = reference;

    /* Calculate integral windup. */
    this->integralWindup = PositionController::codegenIntegralWindup(
        this->integralWindup, reference, this->stateEstimate,
        configManager.getControllerConfiguration());

    /* Calculate control signal (unclamped). */
    this->controlSignal = PositionController::codegenControlSignal(
        this->stateEstimate, reference, this->integralWindup,
        configManager.getControllerConfiguration());

    /* Clamp control signal. */
    this->clampControlSignal();

    /* Return the updated control signal. */
    return this->controlSignal;
}

PositionControlSignal
PositionController::updateControlSignalBlind(PositionReference reference) {

    /* Save the reference position. */
    this->reference = reference;

    /* Calculate integral windup. */
    this->integralWindup = PositionController::codegenIntegralWindupBlind(
        this->integralWindup, reference, this->stateEstimate,
        configManager.getControllerConfiguration());

    /* Calculate control signal (unclamped). */
    this->controlSignal = PositionController::codegenControlSignalBlind(
        this->stateEstimate, reference, this->integralWindup,
        configManager.getControllerConfiguration());

    /* Clamp control signal. */
    this->clampControlSignal();

    /* Return the updated control signal. */
    return this->controlSignal;
}

void PositionController::updateObserver(Quaternion orientation,
                                        PositionMeasurement measurement) {
    /* Calculate the current state estimate. */
    this->stateEstimate = PositionController::codegenCurrentStateEstimate(
        this->stateEstimate, measurement, orientation,
        getTime() - lastMeasurementTime,
        configManager.getControllerConfiguration());

    /* Store the measurement time. */
    this->lastMeasurementTime = getTime();
}

void PositionController::updateObserverBlind(Quaternion orientation) {

    PositionStateBlind stateBlind = {
        this->stateEstimate.p,
        this->stateEstimate.vx,
        this->stateEstimate.vy,
    };
    PositionControlSignalBlind controlSignalBlind = {orientation[1],
                                                     orientation[2]};

    this->stateEstimate =
        codegenCurrentStateEstimateBlind(stateBlind, controlSignalBlind);
}
