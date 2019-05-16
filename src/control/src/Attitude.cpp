#include <Attitude.hpp>

/* Includes from src. */
#include <MiscInstances.hpp>  ///< ConfigurationManager instance
#include <RCValues.hpp>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>  ///< SECONDS_PER_TICK

#pragma region Constants
/**
 * The largest control signal that can be sent to the "yaw torque motor" is
 * 0.10.
 */
static constexpr real_t YAW_SIGNAL_CLAMP = 0.10;

/**
 * The most the drone can tilt is 0.1745 rad (10 deg). If the hardware constants
 * RC_LOW and RC_HIGH are defined correctly, then the drone's reference roll
 * (pitch) will be 10 degrees when the pilot pushes the tilt stick completely to
 * the right (downwards).
 */
static constexpr real_t MAXIMUM_REFERENCE_TILT = 0.1745;

/** The maximum speed of the reference yaw is 0.80 rad/s. */
static constexpr real_t RC_REFERENCE_YAW_MAX_SPEED = 0.80;

/** The threshold to start decreasing the reference yaw is -0.05. */
static constexpr real_t RC_REFERENCE_YAW_LOWER_THRESHOLD = -0.05;

/** The threshold to start increasing the reference yaw is +0.05. */
static constexpr real_t RC_REFERENCE_YAW_UPPER_THRESHOLD = 0.05;
#pragma endregion

MotorSignals transformAttitudeControlSignal(AttitudeControlSignal controlSignal,
                                            real_t commonThrust) {
    real_t ux = controlSignal.uxyz[0];
    real_t uy = controlSignal.uxyz[1];
    real_t uz = controlSignal.uxyz[2];
    return MotorSignals{commonThrust + ux + uy - uz,  //
                        commonThrust + ux - uy + uz,  //
                        commonThrust - ux + uy + uz,  //
                        commonThrust - ux - uy - uz};
}

void AttitudeController::calculateJumpedQuaternions(real_t yawJumpRads) {
    this->stateEstimate.q = EulerAngles::eul2quat(
        {this->orientationEuler.yaw + yawJumpRads, this->orientationEuler.pitch,
         this->orientationEuler.roll});
    this->reference.q = EulerAngles::eul2quat(
        {this->referenceEuler.yaw + yawJumpRads, this->referenceEuler.pitch,
         this->referenceEuler.roll});
}

void AttitudeController::clampControlSignal(real_t commonThrust) {

    /* Load values from the attitude controller. */
    real_t ux = this->controlSignal.uxyz[0];
    real_t uy = this->controlSignal.uxyz[1];
    real_t uz = this->controlSignal.uxyz[2];

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

    this->controlSignal = AttitudeControlSignal{ux, uy, uz};
}

void AttitudeController::init() {

    /* Reset the attitude controller. */
    this->stateEstimate    = {};
    this->orientationEuler = {};
    this->integralWindup   = {};
    this->controlSignal    = {};
    this->reference        = {};
    this->referenceEuler   = {};
}

AttitudeControlSignal
AttitudeController::updateControlSignal(real_t commonThrust) {

    /* Calculate integral windup. */
    this->integralWindup = AttitudeController::codegenIntegralWindup(
        this->integralWindup, this->reference, this->stateEstimate,
        configManager.getControllerConfiguration());

    /* Calculate control signal (unclamped). */
    this->controlSignal = AttitudeController::codegenControlSignal(
        this->stateEstimate, this->reference, this->integralWindup,
        configManager.getControllerConfiguration());

    /* Clamp control signal. */
    this->clampControlSignal(commonThrust);

    /* Return the updated control signal. */
    return this->controlSignal;
}

void AttitudeController::updateObserver(AttitudeMeasurement measurement,
                                        real_t yawJumpToSubtract) {

    /* Save measurement for logger. */
    this->measurement = measurement;

    /* Update state estimate. */
    this->stateEstimate = AttitudeController::codegenNextStateEstimate(
        this->stateEstimate, this->controlSignal, measurement,
        configManager.getControllerConfiguration());

    /* Update the EulerAngles representation as well! */
    this->orientationEuler = EulerAngles::quat2eul(this->stateEstimate.q);
    this->orientationEuler.yaw -= yawJumpToSubtract;
}

void AttitudeController::updateRCReference() {

    /* Store RC values. */
    real_t roll  = getRoll();
    real_t pitch = getPitch();
    real_t yaw   = getYaw();

    /* Convert RC tilt [-1.0, +1.0] to radians [-0.1745, +0.1745]. */
    real_t rollRads  = roll * MAXIMUM_REFERENCE_TILT;
    real_t pitchRads = pitch * MAXIMUM_REFERENCE_TILT;
    real_t yawRads   = this->referenceEuler.yaw;

    /* Update the reference yaw based on the RC yaw. */
    real_t upperZoneSize = 1.0 - RC_REFERENCE_YAW_UPPER_THRESHOLD;
    real_t lowerZoneSize = RC_REFERENCE_YAW_LOWER_THRESHOLD - (-1.0);
    if (yaw > RC_REFERENCE_YAW_UPPER_THRESHOLD)
        yawRads += (yaw - RC_REFERENCE_YAW_UPPER_THRESHOLD) / upperZoneSize *
                   RC_REFERENCE_YAW_MAX_SPEED * SECONDS_PER_TICK;
    if (yaw < RC_REFERENCE_YAW_LOWER_THRESHOLD)
        yawRads -= (RC_REFERENCE_YAW_LOWER_THRESHOLD - yaw) / lowerZoneSize *
                   RC_REFERENCE_YAW_MAX_SPEED * SECONDS_PER_TICK;

    /* Store the EulerAngles reference orientation. */
    this->referenceEuler = EulerAngles{yawRads, pitchRads, rollRads};
}
