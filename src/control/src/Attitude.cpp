#include <Attitude.hpp>
#include <Configuration.hpp>
#include <Globals.hpp>
#include <Time.hpp>

/**
 * The largest control signal that can be sent to the "yaw torque motor" is
 * 0.10.
 */
const real_t YAW_SIGNAL_CLAMP = 0.10;

/**
 * The most the drone can tilt is 0.1745 rad (10 deg). If the hardware constants
 * RC_LOW and RC_HIGH are defined correctly, then the drone's reference roll
 * (pitch) will be 10 degrees when the pilot pushes the tilt stick completely to
 * the right (downwards).
 */
const real_t MAXIMUM_REFERENCE_TILT = 0.1745;

/** The maximum speed of the reference yaw is 0.80 rad/s. */
const real_t RC_REFERENCE_YAW_MAX_SPEED = 0.80;

/** The threshold to start decreasing the reference yaw is -0.05. */
const real_t RC_REFERENCE_YAW_LOWER_THRESHOLD = -0.05;

/** The threshold to start increasing the reference yaw is +0.05. */
const real_t RC_REFERENCE_YAW_UPPER_THRESHOLD = 0.05;

MotorDutyCycles
transformAttitudeControlSignal(AttitudeControlSignal controlSignal,
                               real_t commonThrust) {
    return MotorDutyCycles{
        commonThrust + controlSignal.ux + controlSignal.uy - controlSignal.uz,
        commonThrust + controlSignal.ux - controlSignal.uy + controlSignal.uz,
        commonThrust - controlSignal.ux + controlSignal.uy + controlSignal.uz,
        commonThrust - controlSignal.ux - controlSignal.uy - controlSignal.uz};
}

void AttitudeController::calculateJumpedQuaternions(real_t yawJumpRads) {
    this->stateEstimate.q = EulerAngles::eul2quat(
        {this->orientationEuler.yaw + yawJumpRads, this->orientationEuler.pitch,
         this->orientationEuler.roll});
    this->reference.q = EulerAngles::eul2quat(
        {this->referenceEuler.yaw + yawJumpRads, this->referenceEuler.pitch,
         this->referenceEuler.roll});
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

void AttitudeController::init() {

    /* Reset the attitude controller. */
    this->stateEstimate    = {};
    this->orientationEuler = EulerAngles({});
    this->integralWindup   = {};
    this->controlSignal    = {};
    this->reference        = {};
    this->referenceEuler   = EulerAngles({});
}

AttitudeControlSignal
AttitudeController::updateControlSignal(real_t commonThrust) {

    /* Calculate integral windup. */
    this->integralWindup =
        codegenIntegralWindup(this->integralWindup, this->reference,
                              this->stateEstimate, getDroneConfiguration());

    /* Calculate control signal (unclamped). */
    this->controlSignal =
        codegenControlSignal(this->stateEstimate, this->reference,
                             this->integralWindup, getDroneConfiguration());

    /* Clamp control signal. */
    this->controlSignal = clampControlSignal(this->controlSignal, commonThrust);

    return this->controlSignal;
}

void AttitudeController::updateObserver(AttitudeMeasurement measurement,
                                        real_t yawJumpToSubtract) {
    this->stateEstimate =
        codegenNextStateEstimate(this->stateEstimate, this->controlSignal,
                                 measurement, getDroneConfiguration());

    /* Update the EulerAngles representation as well! */
    this->orientationEuler = EulerAngles::quat2eul(this->stateEstimate.q);
    this->orientationEuler.yaw -= yawJumpToSubtract;
}

void AttitudeController::updateRCReference() {

    /* Store RC values. */
    real_t roll  = getRCRoll();
    real_t pitch = getRCPitch();
    real_t yaw   = getRCYaw();

    /* Convert RC tilt [-0.5, +0,5] to radians [-0.1745, +0.1745]. */
    real_t rollRads  = 2 * roll * MAXIMUM_REFERENCE_TILT;
    real_t pitchRads = 2 * pitch * MAXIMUM_REFERENCE_TILT;
    real_t yawRads   = this->referenceEuler.yaw;

    /* Update the reference yaw based on the RC yaw. */
    real_t upperZoneSize = 0.5 - RC_REFERENCE_YAW_UPPER_THRESHOLD;
    real_t lowerZoneSize = RC_REFERENCE_YAW_LOWER_THRESHOLD - (-0.5);
    if (yaw > RC_REFERENCE_YAW_UPPER_THRESHOLD)
        yawRads += (yaw - RC_REFERENCE_YAW_UPPER_THRESHOLD) / upperZoneSize *
                   RC_REFERENCE_YAW_MAX_SPEED * SECONDS_PER_TICK;
    if (yaw < RC_REFERENCE_YAW_LOWER_THRESHOLD)
        yawRads -= (RC_REFERENCE_YAW_LOWER_THRESHOLD - yaw) / lowerZoneSize *
                   RC_REFERENCE_YAW_MAX_SPEED * SECONDS_PER_TICK;

    /* Store the EulerAngles reference orientation. */
    this->referenceEuler = EulerAngles{yawRads, pitchRads, rollRads};
}
