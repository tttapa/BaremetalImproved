#include "../../../src-vivado/main/include/PublicHardwareConstants.hpp"
#include <Altitude.hpp>
#include <MiscInstances.hpp>
#include <RCValues.hpp>

/**
 * The largest marginal control signal that can be sent to the "common motor"
 * is 0.10.
 */
static constexpr real_t MARGINAL_SIGNAL_CLAMP = 0.10;

/** The maximum height at which the drone may hover is 1.75 meters. */
static constexpr real_t MAXIMUM_REFERENCE_HEIGHT = 1.75;

/** The minimum height at which the drone may hover is 0.25 meters. */
static constexpr real_t MINIMUM_REFERENCE_HEIGHT = 0.25;

/** The maximum speed of the reference height is 0.25 m/s. */
static constexpr real_t RC_HEIGHT_REFERENCE_MAX_SPEED = 0.25;

/** The threshold to start decreasing the reference height is 0.25. */
static constexpr real_t RC_REFERENCE_HEIGHT_LOWER_THRESHOLD = 0.25;

/** The threshold to start increasing the reference height is 0.75. */
static constexpr real_t RC_REFERENCE_HEIGHT_UPPER_THRESHOLD = 0.75;

AltitudeControlSignal
AltitudeController::clampControlSignal(AltitudeControlSignal controlSignal) {
    if (controlSignal.ut > MARGINAL_SIGNAL_CLAMP)
        return AltitudeControlSignal{MARGINAL_SIGNAL_CLAMP};
    if (controlSignal.ut < -MARGINAL_SIGNAL_CLAMP)
        return AltitudeControlSignal{-MARGINAL_SIGNAL_CLAMP};
    return AltitudeControlSignal{controlSignal.ut};
}

void AltitudeController::init() {

    /* Reset the altitude controller. */
    this->controlSignal  = {};
    this->integralWindup = {};
    this->stateEstimate  = {};
    this->reference      = {};
}

void AltitudeController::setReference(AltitudeReference reference) {
    this->reference = reference;
}

AltitudeControlSignal AltitudeController::updateControlSignal() {

    /* Calculate integral windup. */
    this->integralWindup = codegenIntegralWindup(
        this->integralWindup, this->reference, this->stateEstimate,
        configManager.getControllerConfiguration());

    /* Calculate control signal (unclamped). */
    this->controlSignal = codegenControlSignal(
        this->stateEstimate, this->reference, this->integralWindup,
        configManager.getControllerConfiguration());

    /* Clamp control signal. */
    this->controlSignal = clampControlSignal(this->controlSignal);

    return this->controlSignal;
}

void AltitudeController::updateObserver(AltitudeMeasurement measurement) {
    this->stateEstimate = codegenNextStateEstimate(
        this->stateEstimate, this->controlSignal, measurement,
        configManager.getControllerConfiguration());
}

void AltitudeController::updateRCReference() {

    /* Store the RC throttle. */
    real_t throttle = getThrottle();

    /* Try increasing/decreasing the reference height. */
    real_t upperZoneSize = 1.0 - RC_REFERENCE_HEIGHT_UPPER_THRESHOLD;
    real_t lowerZoneSize = RC_REFERENCE_HEIGHT_LOWER_THRESHOLD;
    if (throttle > RC_REFERENCE_HEIGHT_UPPER_THRESHOLD)
        this->reference.z += (throttle - RC_REFERENCE_HEIGHT_UPPER_THRESHOLD) /
                             upperZoneSize * RC_HEIGHT_REFERENCE_MAX_SPEED /
                             SONAR_FREQUENCY;
    if (throttle < RC_REFERENCE_HEIGHT_LOWER_THRESHOLD)
        this->reference.z -= (RC_REFERENCE_HEIGHT_LOWER_THRESHOLD - throttle) /
                             lowerZoneSize * RC_HEIGHT_REFERENCE_MAX_SPEED /
                             SONAR_FREQUENCY;

    /* Clamp the reference height. */
    if (this->reference.z < MINIMUM_REFERENCE_HEIGHT)
        this->reference.z = MINIMUM_REFERENCE_HEIGHT;
    if (this->reference.z > MAXIMUM_REFERENCE_HEIGHT)
        this->reference.z = MAXIMUM_REFERENCE_HEIGHT;
}