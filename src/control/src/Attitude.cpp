#include <Attitude.hpp>
#include <Configuration.hpp>
#include <Globals.hpp>

/**
 * The largest control signal that can be sent to the "yaw torque motor" is
 * 0.10.
 */
const real_t YAW_SIGNAL_CLAMP = 0.10;

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
    if (uz > YAW_SIGNAL_CLAMP)
        uz = YAW_SIGNAL_CLAMP;
    if (uz < -YAW_SIGNAL_CLAMP)
        uz = -YAW_SIGNAL_CLAMP;

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

Quaternion AttitudeController::getOrientationEstimate() {
    return this->stateEstimate.q;
}

void AttitudeController::init() {

    /* Reset the attitude controller. */
    this->stateEstimate  = {};
    this->integralWindup = {};
    this->controlSignal  = {};
}

AttitudeControlSignal
AttitudeController::updateControlSignal(AttitudeReference reference,
                                        real_t commonThrust) {

    /* Calculate integral windup. */
    this->integralWindup = codegenIntegralWindup(
        this->integralWindup, reference, getDroneConfiguration());

    /* Calculate control signal (unclamped). */
    this->controlSignal =
        codegenControlSignal(this->stateEstimate, reference,
                             this->integralWindup, getDroneConfiguration());

    /* Clamp control signal. */
    this->controlSignal = clampControlSignal(this->controlSignal, commonThrust);

    return this->controlSignal;
}

void AttitudeController::updateObserver(AttitudeMeasurement measurement) {
    this->stateEstimate =
        codegenNextStateEstimate(this->stateEstimate, this->controlSignal,
                                 measurement, getDroneConfiguration());
}
