#include <Altitude.hpp>

/* Includes from src. */
#include <MiscInstances.hpp>  ///< ConfigurationManager instance
#include <RCValues.hpp>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>  ///< SONAR_FREQUENCY

#pragma region Constants
/**
 * The largest marginal control signal that can be sent to the "common motor"
 * is 0.04.
 */
static constexpr float MARGINAL_SIGNAL_CLAMP = 0.04;

/** The maximum height at which the drone may hover is 1.75 meters. */
static constexpr float MAXIMUM_REFERENCE_HEIGHT = 1.75;

/** The minimum height at which the drone may hover is 0.25 meters. */
static constexpr float MINIMUM_REFERENCE_HEIGHT = 0.25;

/** The maximum speed of the reference height is 0.25 m/s. */
static constexpr float RC_HEIGHT_REFERENCE_MAX_SPEED = 0.25;

/** The threshold to start decreasing the reference height is 0.20. */
static constexpr float RC_REFERENCE_HEIGHT_LOWER_THRESHOLD = 0.20;

/** The threshold to start increasing the reference height is 0.80. */
static constexpr float RC_REFERENCE_HEIGHT_UPPER_THRESHOLD = 0.80;
#pragma endregion

void AltitudeController::clampControlSignal() {
    if (this->controlSignal.ut > MARGINAL_SIGNAL_CLAMP)
        this->controlSignal.ut = MARGINAL_SIGNAL_CLAMP;
    if (this->controlSignal.ut < -MARGINAL_SIGNAL_CLAMP)
        this->controlSignal.ut = -MARGINAL_SIGNAL_CLAMP;
}

void AltitudeController::init(float correctedMeasurementHeight) {

    /* Reset the altitude controller. */
    this->controlSignal  = {};
    this->integralWindup = {};
    this->stateEstimate  = {0.0, correctedMeasurementHeight, 0.0};
    this->reference      = {correctedMeasurementHeight};
}

void AltitudeController::setReference(AltitudeReference reference) {
    this->reference = reference;
}

AltitudeControlSignal AltitudeController::updateControlSignal() {

    /* Calculate integral windup. */
    this->integralWindup = AltitudeController::codegenIntegralWindup(
        this->integralWindup, this->reference, this->stateEstimate,
        configManager.getControllerConfiguration());

    /* Calculate control signal (unclamped). */
    this->controlSignal = AltitudeController::codegenControlSignal(
        this->stateEstimate, this->reference, this->integralWindup,
        configManager.getControllerConfiguration());

    /* Clamp control signal. */
    this->clampControlSignal();

    /* Return the updated control signal. */
    return this->controlSignal;
}

void AltitudeController::updateObserver(AltitudeMeasurement measurement) {

    /* Save measurement for logger. */
    this->measurement = measurement;

    /* Update state estimate. */
    this->stateEstimate = AltitudeController::codegenNextStateEstimate(
        this->stateEstimate, this->controlSignal, measurement,
        configManager.getControllerConfiguration());
}

void AltitudeController::updateRCReference() {

    /* Store the RC throttle. */
    float throttle = getThrottle();

    /* Try increasing/decreasing the reference height. */
    float upperZoneSize = 1.0 - RC_REFERENCE_HEIGHT_UPPER_THRESHOLD;
    float lowerZoneSize = RC_REFERENCE_HEIGHT_LOWER_THRESHOLD;
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