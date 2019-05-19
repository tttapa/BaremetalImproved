#include <Attitude.hpp>

/* Includes from src. */
#include <MiscInstances.hpp>  ///< ConfigurationManager instance
#include <RCValues.hpp>
#include <MathFunctions.hpp>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>  ///< SECONDS_PER_TICK

#pragma region Constants
/**
 * The largest control signal that can be sent to the "yaw torque motor" is
 * 0.08.
 */
static constexpr float YAW_SIGNAL_CLAMP = 0.08;

/**
 * The most the drone can tilt is 0.1745 rad (10 deg). If the hardware constants
 * RC_LOW and RC_HIGH are defined correctly, then the drone's reference roll
 * (pitch) will be 10 degrees when the pilot pushes the tilt stick completely to
 * the right (downwards).
 */
static constexpr float MAXIMUM_REFERENCE_TILT = 0.1745;

/** The maximum speed of the reference yaw is 0.80 rad/s. */
static constexpr float RC_REFERENCE_YAW_MAX_SPEED = 0.80;

/** The threshold to start decreasing the reference yaw is -0.05. */
static constexpr float RC_REFERENCE_YAW_LOWER_THRESHOLD = -0.05;

/** The threshold to start increasing the reference yaw is +0.05. */
static constexpr float RC_REFERENCE_YAW_UPPER_THRESHOLD = 0.05;
#pragma endregion

MotorSignals transformAttitudeControlSignal(AttitudeControlSignal controlSignal,
                                            float commonThrust) {
    float ux = controlSignal.uxyz.x;
    float uy = controlSignal.uxyz.y;
    float uz = controlSignal.uxyz.z;
    return MotorSignals{commonThrust + ux + uy - uz,  //
                        commonThrust + ux - uy + uz,  //
                        commonThrust - ux + uy + uz,  //
                        commonThrust - ux - uy - uz};
}

void AttitudeController::calculateJumpedQuaternions(float yawJumpRads) {
    this->stateEstimate.q = EulerAngles::eul2quat(
        {this->orientationEuler.yaw + yawJumpRads, this->orientationEuler.pitch,
         this->orientationEuler.roll});
    this->reference.q = EulerAngles::eul2quat(
        {this->referenceEuler.yaw + yawJumpRads, this->referenceEuler.pitch,
         this->referenceEuler.roll});
}

void AttitudeController::clampControlSignal(float commonThrust) {

    /* Load values from the attitude controller. */
    float ux = this->controlSignal.uxyz.x;
    float uy = this->controlSignal.uxyz.y;
    float uz = this->controlSignal.uxyz.z;

    /* Clamp the yaw torque motor separately to ensure ux, uy compensation. */
    if (uz > YAW_SIGNAL_CLAMP)
        uz = YAW_SIGNAL_CLAMP;
    if (uz < -YAW_SIGNAL_CLAMP)
        uz = -YAW_SIGNAL_CLAMP;

    /* Clamp ux, uy, uz such that all motor PWM duty cycles are in [0,1]. */
    // TODO: divide by e = epsilon + 1?
    float absoluteSum    = std2::absf(ux) + std2::absf(uy) + std2::absf(uz);
    float maxAbsoluteSum = std2::minf(commonThrust, 1 - commonThrust);
    if (absoluteSum > maxAbsoluteSum) {
        float factor = maxAbsoluteSum / absoluteSum;
        ux *= factor;
        uy *= factor;
        uz *= factor;
    }

    this->controlSignal = AttitudeControlSignal{Vec3f{ux, uy, uz}};
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
AttitudeController::updateControlSignal(float commonThrust) {

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
                                        float yawJumpToSubtract) {

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
    float roll  = getRoll();
    float pitch = getPitch();
    float yaw   = getYaw();

    /* Convert RC tilt [-1.0, +1.0] to radians [-0.1745, +0.1745]. */
    float rollRads  = roll * MAXIMUM_REFERENCE_TILT;
    float pitchRads = pitch * MAXIMUM_REFERENCE_TILT;
    float yawRads   = this->referenceEuler.yaw;

    /* Update the reference yaw based on the RC yaw. */
    float upperZoneSize = 1.0 - RC_REFERENCE_YAW_UPPER_THRESHOLD;
    float lowerZoneSize = RC_REFERENCE_YAW_LOWER_THRESHOLD - (-1.0);
    if (yaw > RC_REFERENCE_YAW_UPPER_THRESHOLD)
        yawRads += (yaw - RC_REFERENCE_YAW_UPPER_THRESHOLD) / upperZoneSize *
                   RC_REFERENCE_YAW_MAX_SPEED * SECONDS_PER_TICK;
    if (yaw < RC_REFERENCE_YAW_LOWER_THRESHOLD)
        yawRads -= (RC_REFERENCE_YAW_LOWER_THRESHOLD - yaw) / lowerZoneSize *
                   RC_REFERENCE_YAW_MAX_SPEED * SECONDS_PER_TICK;

    /* Store the EulerAngles reference orientation. */
    this->referenceEuler = EulerAngles{yawRads, pitchRads, rollRads};
    this->reference = EulerAngles::eul2quat(this->referenceEuler);
}
