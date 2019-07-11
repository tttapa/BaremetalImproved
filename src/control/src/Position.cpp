#include <Position.hpp>

/* Includes from src. */
#include <MiscInstances.hpp>
#include <Time.hpp>

/**
 * The largest reference quaternion component that can be sent to the attitude
 * control system is 3 degrees:
 *  1°: 0.0087
 *  2°: 0.0175
 *  3°: 0.0262
 *  4°: 0.0349
 *  5°: 0.0436
 *  6°: 0.0523  
 */
static constexpr float REFERENCE_QUATERNION_CLAMP = 0.0262;

float dist(Position a, Position b) { return Vec2f::norm(b - a); }

float distsq(Position a, Position b) { return Vec2f::normsq(b - a); }

void PositionController::clampControlSignal() {
    if (this->controlSignal.q12.x > REFERENCE_QUATERNION_CLAMP)
        this->controlSignal.q12.x = REFERENCE_QUATERNION_CLAMP;
    if (this->controlSignal.q12.x < -REFERENCE_QUATERNION_CLAMP)
        this->controlSignal.q12.x = -REFERENCE_QUATERNION_CLAMP;
    if (this->controlSignal.q12.y > REFERENCE_QUATERNION_CLAMP)
        this->controlSignal.q12.y = REFERENCE_QUATERNION_CLAMP;
    if (this->controlSignal.q12.y < -REFERENCE_QUATERNION_CLAMP)
        this->controlSignal.q12.y = -REFERENCE_QUATERNION_CLAMP;
}

void PositionController::init(Position currentPosition) {

    /* Reset the position controller. */
    this->stateEstimate       = {{}, currentPosition, {}};
    this->integralWindup      = {};
    this->controlSignal       = {};
    this->lastMeasurementTime = getTime();
    this->posFilt = currentPosition;
}

void PositionController::correctPositionEstimateBlocks(
    Position correctPosition) {
    Position deltaBlocks = correctPosition - stateEstimate.p * METERS_TO_BLOCKS;
    Position offsetBlocks = deltaBlocks.round();
    this->stateEstimate.p = stateEstimate.p + offsetBlocks * BLOCKS_TO_METERS;
}

void PositionController::correctPositionEstimateBlocks(
    VisionPosition correctPosition) {
    correctPositionEstimateBlocks(
        Position{correctPosition.x, correctPosition.y});
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

    /* Save measurement for logger. */
    this->measurement = measurement;

    /* Calculate the current state estimate. */
    this->stateEstimate = PositionController::codegenCurrentStateEstimate(
        this->stateEstimate, measurement, orientation,
        getTime() - lastMeasurementTime,
        configManager.getControllerConfiguration());

    /* Store the measurement time. */
    this->lastMeasurementTime = getTime();
}

void PositionController::updateObserverBlind(Quaternion orientation) {

    /* Update state estimate. */
    this->stateEstimate = codegenCurrentStateEstimateBlind(
        this->stateEstimate, this->controlSignal, orientation);

    /* Store the measurement time. */
    this->lastMeasurementTime = getTime();
}
