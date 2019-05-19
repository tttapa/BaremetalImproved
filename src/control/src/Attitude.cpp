#include <Attitude.hpp>

/* Includes from src. */
#include <EulerAngles.hpp>
#include <MathFunctions.hpp>
#include <MiscInstances.hpp>  ///< ConfigurationManager instance
#include <RCValues.hpp>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>  ///< SECONDS_PER_TICK

#pragma region Constants
/**
 * Whenever the yaw passes 10 degrees (0.1745 rad), it will jump to -10 degrees
 * and vice versa.
 */
static constexpr float MAX_YAW_RADS = 0.1745;

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

float AttitudeController::calculateYawJump() {
    float yaw    = EulerAngles::quat2eul(this->stateEstimate.q).yaw;
    float result = 0;
    while (yaw > MAX_YAW_RADS) {
        yaw -= 2 * MAX_YAW_RADS;
        result -= 2 * MAX_YAW_RADS;
    }
    while (yaw < -MAX_YAW_RADS) {
        yaw += 2 * MAX_YAW_RADS;
        result += 2 * MAX_YAW_RADS;
    }
    return result;
}

Quaternion AttitudeController::calculateDiffQuat() {

    /* Convert orientation estimate to EulerAngles. */
    EulerAngles eul = EulerAngles::quat2eul(this->stateEstimate.q);

    /* If the yaw is in [-MAX_YAW, +MAX_YAW], return the identity quaternion. */
    if (eul.yaw >= -MAX_YAW_RADS && eul.yaw <= -MAX_YAW_RADS)
        return Quaternion::identity();

    /* Definition of pi. */
    const float PI = 3.14159265358979323846;

    /* Otherwise, calculate the quaternion used to rotate the given orientation
       estimate to one where the yaw is in the interval [-MAX_YAW, +MAX_YAW]. */
    while (eul.yaw > MAX_YAW_RADS) {
        eul.yaw -= 2 * MAX_YAW_RADS;
        this->rcReferenceYaw -= 2 * MAX_YAW_RADS;
        if(this->rcReferenceYaw < -PI)
            this->rcReferenceYaw += 2*PI;
    }
    while (eul.yaw < -MAX_YAW_RADS) {
        eul.yaw += 2 * MAX_YAW_RADS;
        this->rcReferenceYaw += 2 * MAX_YAW_RADS;
        if(this->rcReferenceYaw > PI)
            this->rcReferenceYaw -= 2*PI;
    }

    Quaternion diffQuat = EulerAngles::eul2quat(eul) - this->stateEstimate.q;

    /* Rotate the state estimate and the reference by this quaternion. The AHRS
       will also be rotated by it. This way we'll keep the orientation estimate
       near the identity quaternion, but the controller will still be
       accurate. */
    this->stateEstimate.q = diffQuat + this->stateEstimate.q;
    this->reference.q     = diffQuat + this->reference.q;

    /* Return diffQuat, so the AHRS can update its orientation too. */
    return diffQuat;
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

float AttitudeController::getRCPitchRads() {
    /* Convert RC pitch [-1.0, +1.0] to radians [-0.1745, +0.1745]. */
    return getPitch() * MAXIMUM_REFERENCE_TILT;
}

float AttitudeController::getRCRollRads() {
    /* Convert RC roll [-1.0, +1.0] to radians [-0.1745, +0.1745]. */
    return getRoll() * MAXIMUM_REFERENCE_TILT;
}

void AttitudeController::init() {

    /* Reset the attitude controller. */
    this->stateEstimate  = {};
    this->integralWindup = {};
    this->controlSignal  = {};
    this->reference      = {};
    this->rcReferenceYaw = 0.0;
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

void AttitudeController::updateObserver(AttitudeMeasurement measurement) {

    /* Save measurement for logger. */
    this->measurement = measurement;

    /* Update state estimate. */
    this->stateEstimate = AttitudeController::codegenNextStateEstimate(
        this->stateEstimate, this->controlSignal, measurement,
        configManager.getControllerConfiguration());
}

float AttitudeController::updateRCYawRads() {

    /* Store RC values. */
    float yaw = getYaw();

    /* Convert RC tilt [-1.0, +1.0] to radians [-0.1745, +0.1745]. */
    float yawRads = this->rcReferenceYaw;

    /* Update the reference yaw based on the RC yaw. */
    float upperZoneSize = 1.0 - RC_REFERENCE_YAW_UPPER_THRESHOLD;
    float lowerZoneSize = RC_REFERENCE_YAW_LOWER_THRESHOLD - (-1.0);
    if (yaw > RC_REFERENCE_YAW_UPPER_THRESHOLD)
        yawRads += (yaw - RC_REFERENCE_YAW_UPPER_THRESHOLD) / upperZoneSize *
                   RC_REFERENCE_YAW_MAX_SPEED * SECONDS_PER_TICK;
    if (yaw < RC_REFERENCE_YAW_LOWER_THRESHOLD)
        yawRads -= (RC_REFERENCE_YAW_LOWER_THRESHOLD - yaw) / lowerZoneSize *
                   RC_REFERENCE_YAW_MAX_SPEED * SECONDS_PER_TICK;

    /* Update the reference. */
    this->rcReferenceYaw = yawRads;
    return this->rcReferenceYaw;


}
