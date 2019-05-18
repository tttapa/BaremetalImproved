#pragma once

/* Includes from src. */
#include <EulerAngles.hpp>
#include <LoggerStructs.hpp>
#include <Quaternion.hpp>

/**
 * Transform the given attitude control signal to the duty cycles to be sent to
 * the ESCs of the four motors.
 * 
 * @param   controlSignal
 *          The attitude control signal to transform.
 * @param   commonThrust
 *          The common thrust to transform.
 * 
 * @return  The duty cycles to the four motors.
 */
MotorSignals transformAttitudeControlSignal(AttitudeControlSignal controlSignal,
                                            float commonThrust);

/**
 * Class to control the attitude of the drone. The first part is an observer to
 * estimate the drone's orientation, angular velocity and the angular velocity
 * of the "torque motors". Next, there is a controller to send appropriate PWM
 * signals to the torque motors based on how far the drone's state estimate
 * deviates from the reference state.
 *
 * To achieve this, the AttitudeController contains variables to store the
 * state estimate, integral windup and control signal.
 */
class AttitudeController {

  private:
    /** PWM control signals sent to the torque motors (ux, uy, uz). */
    AttitudeControlSignal controlSignal;

    /** Integral of the error of the quaternion components q1, q2 and q3. */
    AttitudeIntegralWindup integralWindup;

    /**
     * Estimate of the drone's orientation as EulerAngles. This representation
     * facilitates the quaternion jumps when the state estimate's yaw becomes
     * too large.
     */
    EulerAngles orientationEuler;

    /** Measurement orientation and angular velocity from the IMU and AHRS. */
    AttitudeMeasurement measurement;

    /** Reference orientation to track. */
    AttitudeReference reference;

    /**
     * Reference orientation to track as EulerAngles. This is used to keep track
     * of the reference yaw, which needs to be remembered between clock cycles.
     * Also, this facilitates the quaternion jumps when the state estimate's yaw
     * becomes too large, and this data will passed on to the logger.
     */
    EulerAngles referenceEuler;

    /**
     * Estimate of the state of the drone's attitude, consisting of the drone's
     * orientation (1 quaternion), angular velocity in rad/s (3 components: wx,
     * wy, wz) and the angular velocity of the torque motors in rad/s (3
     * components: nx, ny, nz).
     */
    AttitudeState stateEstimate;

  public:
    /**
     * Using the given yaw jump, calculate the quaternion representation of the
     * drone's orientation and the reference orientation. Then store these in
     * the state estimate and in the controller's reference. This "yaw jumping"
     * is used to keep the state estimate's orientation near the unit quaternion
     * [1;0;0;0] in order to ensure the control system's stability.
     * 
     * @param   yawJumpRads
     *          Radians to add to the EulerAngles representation of the drone's
     *          orientation and the reference orientation.
     */
    void calculateJumpedQuaternions(float yawJumpRads);

    /**
     * Clamp the current attitude control signal such that the corrections are
     * not dominated by the yaw component and such that each motor PWM duty
     * cycle is in [0,1].
     * 
     * @param   commonThrust
     *          Control signal to be sent to the "common motor": this must be in
     *          [0,1].
     */
    void clampControlSignal(float commonThrust);

    /**
     * Calculate the current attitude control signal using the code generator.
     * 
     * @param   stateEstimate
     *          Estimate of the current state, determined last cycle.
     * @param   reference
     *          Reference orientation to track.
     * @param   integralWindup
     *          Current integral windup.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The control signal to be sent to the "torque motors" until the
     *          next IMU measurement.
     */
    static AttitudeControlSignal codegenControlSignal(
        AttitudeState stateEstimate, AttitudeReference reference,
        AttitudeIntegralWindup integralWindup, int droneConfiguration);

    /**
     * Calculate the current integral windup using the code generator.
     * 
     * @param   lastIntegralWindup
     *          Integral windup from the last cycle.
     * @param   reference
     *          Reference orientation to track.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The current integral windup.
     */
    static AttitudeIntegralWindup
    codegenIntegralWindup(AttitudeIntegralWindup integralWindup,
                          AttitudeReference reference,
                          AttitudeState stateEstimate, int droneConfiguration);

    /**
     * Calculate the next attitude estimate using the code generator. Because
     * the attitude control system is implemented with a Kalman filter, this
     * function should be called after AttitudeController::
     * codegenControlSignal() is called in order to determine the state estimate
     * for the next cycle.
     * 
     * @param   stateEstimate
     *          Estimate of the current state, determined last cycle.
     * @param   controlSignal
     *          Control signal that will be sent to the "torque motors" until
     *          the next IMU measurement.
     * @param   measurement
     *          Current measurement from the IMU.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The estimate of the next attitude state.
     */
    static AttitudeState codegenNextStateEstimate(
        AttitudeState stateEstimate, AttitudeControlSignal controlSignal,
        AttitudeMeasurement measurement, int droneConfiguration);

    /** Get the attitude controller's control signal. */
    AttitudeControlSignal getControlSignal() { return this->controlSignal; }

    /** Get the attitude controller's integral windup. */
    AttitudeIntegralWindup getIntegralWindup() { return this->integralWindup; }

    /** Get the attitude controller's measurement. */
    AttitudeMeasurement getMeasurement() { return this->measurement; }

    /**
     * Returns the quaternion of the attitude controller's estimate of the
     * drone's orientation. This value is "jumped" in order to keep the estimate
     * near the unit quaternion [1;0;0;0].
     */
    Quaternion getOrientationQuat() { return this->stateEstimate.q; }

    /**
     * Returns the EulerAngles representation of the attitude controller's
     * estimate of the drone's orientation. This representation facilitates the
     * quaternion jumps when the state estimate's yaw becomes too large.
     */
    EulerAngles getOrientationEuler() { return this->orientationEuler; }

    /** Get the attitude controller's reference. */
    AttitudeReference getReference() { return this->reference; }

    /**
     * Returns the quaternion representation of the reference orientation. This
     * value is "jumped" in order to keep the estimate near the unit quaternion
     * [1;0;0;0].
     */
    Quaternion getReferenceQuat() { return this->reference.q; }

    /**
     * Returns the Euler representation of the reference orientation. This is
     * used to keep track of the reference yaw, which needs to be remembered
     * between clock cycles. Also, this facilitates the quaternion jumps when
     * the state estimate's yaw becomes too large, and this data will passed on
     * to the logger.
     */
    EulerAngles getReferenceEuler() { return this->referenceEuler; }

    /**
     * Get the attitude controller's state estimate.
     */
    AttitudeState getStateEstimate() { return this->stateEstimate; }

    /**
     * Reset the attitude controller to the initial state.
     */
    void init();

    /**
     * Set the attitude controller's EulerAngles orientation estimate to the
     * given EulerAngles. This representation facilitates the quaternion jumps
     * when the state estimate's yaw becomes too large.
     * 
     * @param   orientationEuler
     *          New orientation estimate as EulerAngles.
     */
    void setOrientationEuler(EulerAngles orientationEuler) {
        this->orientationEuler = orientationEuler;
    }

    /** Set the attitude controller's reference. */
    void setReference(AttitudeReference reference) { this->reference = reference; }

    /**
     * Set the attitude controller's EulerAngles reference orientation to the
     * given EulerAngles. This is used to keep track of the reference yaw, which
     * needs to be remembered between clock cycles. Also, this facilitates the
     * quaternion jumps when the state estimate's yaw becomes too large, and
     * this data will passed on to the logger.
     * 
     * @param   referenceEuler
     *          New reference orientation to track as EulerAngles. 
     */
    void setReferenceEuler(EulerAngles referenceEuler) {
        this->referenceEuler = referenceEuler;
    }

    /**
     * Update the attitude controller with its current reference orientation.
     * This function should be called when the IMU receives a new measurement.
     * 
     * @param   commonThrust
     *          Control signal sent to the "common motor".
     *
     * @return  The control signal to be sent to the "torque motors" until the
     *          next IMU measurement.
     */
    AttitudeControlSignal updateControlSignal(float commonThrust);

    /**
     * Update the attitude observer with the given IMU measurement. This
     * function should be called when the IMU receives a new measurement.
     * Because the attitude control system is implemented with a Kalman filter,
     * this function should be called after AttitudeController::
     * updateControlSignal() is called in order to determine the state estimate
     * for the next cycle.
     * 
     * @param   measurement
     *          New measurement from the IMU.
     * @param   yawJumpToSubtract
     *          Yaw jump calculated in the beginning of the clock cycle.
     */
    void updateObserver(AttitudeMeasurement measurement,
                        float yawJumpToSubtract);

    /**
     * Update the attitude controller's reference orientation using the RC
     * pitch, roll and yaw. The pitch and roll will be added to the input bias
     * to attain the total reference pitch and roll. The reference yaw will be
     * incremented or decremented by a small amount based on the RC yaw. When
     * the RC yaw is in the dead zone [-5%, +5%], the reference yaw will not
     * change. If the value of the RC yaw exceeds +5% (goes below -5%), then the
     * reference yaw will increase (decrease). The maximum increase (decrease)
     * speed is reached when the value of the RC yaw reaches +50% (-50%).
     */
    void updateRCReference();
};
