#include <Attitude.hpp>
#include <Configuration.hpp>
#include <Globals.hpp>
#include <SoftwareConstants.hpp>

/* Use software constants from the ATTITUDE namespace. */
using namespace ATTITUDE;

MotorDutyCycles
transformAttitudeControlSignal(AttitudeControlSignal controlSignal,
                               real_t commonThrust) {
    return MotorDutyCycles{
        commonThrust + controlSignal.ux + controlSignal.uy - controlSignal.uz,
        commonThrust + controlSignal.ux - controlSignal.uy + controlSignal.uz,
        commonThrust - controlSignal.ux + controlSignal.uy + controlSignal.uz,
        commonThrust - controlSignal.ux - controlSignal.uy - controlSignal.uz};
}

AttitudeControlSignal
AttitudeController::clampControlSignal(AttitudeControlSignal controlSignal,
                                       real_t commonThrust) {

    /* Load values from the attitude controller. */
    real_t ux = controlSignal.ux;
    real_t uy = controlSignal.uy;
    real_t uz = controlSignal.uz;

    /* Clamp the yaw torque motor separately to ensure ux, uy compensation. */
    if (uz > getYawSignalClamp())
        uz = getYawSignalClamp();
    if (uz < -getYawSignalClamp())
        uz = -getYawSignalClamp();

    /* Clamp ux, uy, uz such that all motor PWM duty cycles are in [0,1]. */
    // TODO: divide by e = epsilon + 1?
    real_t absoluteSum    = std::abs(ux) + std::abs(uy) + std::abs(uz);
    real_t maxAbsoluteSum = std::min(commonThrust, 1 - commonThrust);
    if (absoluteSum > maxAbsoluteSum) {
        real_t factor = maxAbsoluteSum / absoluteSum;
        ux *= factor;
        uy *= factor;
        uz *= factor;
    }

    return AttitudeControlSignal{ux, uy, uz};
}

void AttitudeController::init() {

    /* Reset the attitude controller. */
    AttitudeController::stateEstimate  = {};
    AttitudeController::integralWindup = {};
    AttitudeController::controlSignal  = {};
}

AttitudeControlSignal
AttitudeController::updateControlSignal(AttitudeReference reference,
                                        real_t commonThrust) {

    /* Calculate integral windup. */
    AttitudeController::integralWindup =
        codegenIntegralWindup(AttitudeController::integralWindup, reference);

    /* Calculate control signal (unclamped). */
    AttitudeController::controlSignal = codegenControlSignal(
        AttitudeController::stateEstimate, reference,
        AttitudeController::integralWindup, getDroneConfiguration());

    /* Clamp control signal. */
    AttitudeController::controlSignal =
        clampControlSignal(AttitudeController::controlSignal, commonThrust);

    return AttitudeController::controlSignal;
}

void AttitudeController::updateObserver(AttitudeMeasurement measurement) {
    AttitudeController::stateEstimate = codegenNextStateEstimate(
        AttitudeController::stateEstimate, AttitudeController::controlSignal,
        measurement, getDroneConfiguration());
}
