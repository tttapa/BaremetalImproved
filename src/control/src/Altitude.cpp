#include "../../../src-vivado/main/include/PublicHardwareConstants.hpp"
#include <Altitude.hpp>
#include <Configuration.hpp>
#include <Globals.hpp>

/**
 * The largest marginal control signal that can be sent to the "common motor"
 * is 0.10.
 */
const real_t MARGINAL_SIGNAL_CLAMP = 0.10;

/** The maximum height at which the drone may hover is 1.75 meters. */
const real_t MAXIMUM_REFERENCE_HEIGHT = 1.75;

/** The minimum height at which the drone may hover is 0.25 meters. */
const real_t MINIMUM_REFERENCE_HEIGHT = 0.25;

/** The maximum speed of the reference height is 0.25 m/s. */
const real_t RC_REFERENCE_MAX_SPEED = 0.25;

/** The threshold to start decreasing the reference height is 0.25. */
const real_t RC_REFERENCE_LOWER_THRESHOLD = 0.25;

/** The threshold to start increasing the reference height is 0.75. */
const real_t RC_REFERENCE_UPPER_THRESHOLD = 0.75;

AltitudeReference rcUpdateReferenceHeight(AltitudeReference reference) {

    real_t throttle = getRCThrottle();
    real_t z        = reference.z;

    /* Try increasing/decreasing the reference height. */
    if (throttle > RC_REFERENCE_UPPER_THRESHOLD)
        z += (throttle - RC_REFERENCE_UPPER_THRESHOLD) / SONAR_FREQUENCY;
    if (throttle < RC_REFERENCE_LOWER_THRESHOLD)
        z -= (RC_REFERENCE_LOWER_THRESHOLD - throttle) / SONAR_FREQUENCY;

    /* Clamp the reference height. */
    if (z < MINIMUM_REFERENCE_HEIGHT)
        z = MINIMUM_REFERENCE_HEIGHT;
    if (z > MAXIMUM_REFERENCE_HEIGHT
        z = MAXIMUM_REFERENCE_HEIGHT;

    return AltitudeReference{z};
}

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
    AltitudeController::controlSignal  = {};
    AltitudeController::integralWindup = {};
    AltitudeController::stateEstimate  = {};
}

AltitudeControlSignal
AltitudeController::updateControlSignal(AltitudeReference reference) {

    /* Calculate integral windup. */
    AltitudeController::integralWindup =
        codegenIntegralWindup(AltitudeController::integralWindup, reference);

    /* Calculate control signal (unclamped). */
    AltitudeController::controlSignal = codegenControlSignal(
        AltitudeController::stateEstimate, reference,
        AltitudeController::integralWindup, getDroneConfiguration());

    /* Clamp control signal. */
    AltitudeController::controlSignal =
        clampControlSignal(AltitudeController::controlSignal);

    return AltitudeController::controlSignal;
}

void AltitudeController::updateObserver(AltitudeMeasurement measurement) {
    AltitudeController::stateEstimate = codegenNextStateEstimate(
        AltitudeController::stateEstimate, AltitudeController::controlSignal,
        measurement, getDroneConfiguration());
}
