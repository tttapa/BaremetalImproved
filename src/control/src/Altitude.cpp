#include <Altitude.hpp>
#include <Configuration.hpp>
#include <Globals.hpp>
#include <SoftwareConstants.hpp>

/* Use software constants from the ALTITDUE namespace. */
using namespace ALTITUDE;

AltitudeReference rcUpdateReferenceHeight(AltitudeReference reference) {

    real_t throttle = getRCThrottle();
    real_t z        = reference.z;

    /* Try increasing/decreasing the reference height. */
    if (throttle > getRCReferenceUpperThreshold())
        z += (throttle - getRCReferenceUpperThreshold()) / getSonarFrequency();
    if (throttle < getRCReferenceLowerThreshold())
        z -= (getRCReferenceLowerThreshold() - throttle) / getSonarFrequency();

    /* Clamp the reference height. */
    if (z < getMinimumReferenceHeight())
        z = getMinimumReferenceHeight();
    if (z > getMaximumReferenceHeight())
        z = getMaximumReferenceHeight();

    return AltitudeReference{z};
}

AltitudeControlSignal
AltitudeController::clampControlSignal(AltitudeControlSignal controlSignal) {
    if (controlSignal.ut > getMarginalSignalClamp())
        return AltitudeControlSignal{getMarginalSignalClamp()};
    if (controlSignal.ut < -getMarginalSignalClamp())
        return AltitudeControlSignal{-getMarginalSignalClamp()};
    return AltitudeControlSignal{controlSignal.ut};
}

void AltitudeController::updateObserver(AltitudeMeasurement measurement) {
    AltitudeController::stateEstimate = codegenNextStateEstimate(
        AltitudeController::stateEstimate, AltitudeController::controlSignal,
        measurement, getDroneConfiguration());
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

void AltitudeController::init() {

    /* Reset the altitude controller. */
    AltitudeController::controlSignal  = {};
    AltitudeController::integralWindup = {};
    AltitudeController::stateEstimate  = {};
}
