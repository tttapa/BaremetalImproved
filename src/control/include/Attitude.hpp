#pragma once
#include <Quaternion.hpp>
#include <real_t.h>

/**
 * Attitude reference to track, consisting of a single quaternion.
 */
struct AttitudeReference {
    Quaternion q; /* Orientation */
};

/**
 * Measurement from the IMU, consisting of one quaternion for the drone's
 * orientation and three floats for the drone's angular velocity, measured in
 * rad/s.
 */
struct AttitudeMeasurement {
    Quaternion q; /* Orientation */
    real_t wx;    /* X angular velocity (rad/s) */
    real_t wy;    /* Y angular velocity (rad/s) */
    real_t wz;    /* Z angular velocity (rad/s) */
};

/**
 * Estimate of the state of the drone's attitude, consisting of the drone's
 * orientation (1 quaternion q), angular velocity in rad/s (3 components: wx,
 * wy, wz) and the angular velocity of the torque motors in rad/s (3 components:
 * nx, ny, nz).
 */
struct AttitudeState {
    Quaternion q; /* Orientation */
    real_t wx;    /* X angular velocity (rad/s) */
    real_t wy;    /* Y angular velocity (rad/s) */
    real_t wz;    /* Z angular velocity (rad/s) */
    real_t nx;    /* X motor angular velocity (rad/s) */
    real_t ny;    /* Y motor angular velocity (rad/s) */
    real_t nz;    /* Z motor angular velocity (rad/s) */
};

/**
 * Integral of the error of the quaternion components q1, q2 and q3.
 */
struct AttitudeIntegralWindup {
    real_t q1; /* Orientation q1 component */
    real_t q2; /* Orientation q2 component */
    real_t q3; /* Orientation q3 component */
};

/**
 * PWM control signals sent to the torque motors (3 components: ux, uy, uz).
 */
struct AttitudeControlSignal {
    real_t ux; /* X motor signal (/) */
    real_t uy; /* Y motor signal (/) */
    real_t uz; /* Z motor signal (/) */
};

/**
 * Four floats representing the duty cycles to be sent to the four motors
 * (front-left, front-right, back-left, back-right). The four values should
 * be in [0, 1].
 */
struct MotorDutyCycles {
    real_t v0; /* Front-left motor duty cycle */
    real_t v1; /* Front-right motor duty cycle */
    real_t v2; /* Back-left motor duty cycle */
    real_t v3; /* Back-right motor duty cycle */
};

/**
 * Transform the given attitude control signal to the duty cycles to be sent to
 * the ESCs of the four motors.
 * 
 * @param   controlSignal
 *          the attitude control signal to transform
 * @param   commonThrust
 *          the common thrust to transform
 * 
 * @return  the duty cycles to the four motors.
 */
MotorDutyCycles
transformAttitudeControlSignal(AttitudeControlSignal controlSignal,
                               real_t commonThrust);

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
    /**
     * Estimate of the state of the drone's attitude, consisting of the drone's
     * orientation (1 quaternion), angular velocity in rad/s (3 components: wx,
     * wy, wz) and the angular velocity of the torque motors in rad/s (3
     * components: nx, ny, nz).
     */
    AttitudeState stateEstimate;

    /**
     * Integral of the error of the quaternion components q1, q2 and q3.
     */
    AttitudeIntegralWindup integralWindup;

    /**
     * PWM control signals sent to the torque motors (3 components: ux, uy, uz).
     */
    AttitudeControlSignal controlSignal;

    /**
     * Calculate the current attitude control signal using the code generator.
     * 
     * @param   stateEstimate
     *          estimate of the current state, determined last cycle
     * @param   reference
     *          reference orientation to track
     * @param   integralWindup
     *          current integral windup
     * @param   droneConfiguration
     *          configuration of the drone
     * 
     * @return  the control signal to be sent to the "torque motors" until the
     *          next IMU measurement.
     */
    AttitudeControlSignal codegenControlSignal(
        AttitudeState stateEstimate, AttitudeReference reference,
        AttitudeIntegralWindup integralWindup, int droneConfiguration);

    /**
     * Calculate the current integral windup using the code generator.
     * 
     * @param   lastIntegralWindup
     *          integral windup from the last cycle
     * @param   reference
     *          reference orientation to track
     * 
     * @return  the current integral windup.
     */
    AttitudeIntegralWindup
    codegenIntegralWindup(AttitudeIntegralWindup integralWindup,
                          AttitudeReference reference);

    /**
     * Calculate the next attitude estimate using the code generator. Because
     * the attitude control system is implemented with a Kalman filter, this
     * function should be called after AttitudeController::
     * codegenControlSignal() is called in order to determine the state estimate
     * for the next cycle.
     * 
     * @param   stateEstimate
     *          estimate of the current state, determined last cycle
     * @param   controlSignal
     *          control signal that will be sent to the "torque motors" until
     *          the next IMU measurement
     * @param   measurement
     *          current measurement from the IMU
     * @param   droneConfiguration
     *          configuration of the drone
     * 
     * @return  the estimate of the next attitude state.
     */
    AttitudeState codegenNextStateEstimate(AttitudeState stateEstimate,
                                           AttitudeControlSignal controlSignal,
                                           AttitudeMeasurement measurement,
                                           int droneConfiguration);

    /**
     * Clamp the given attitude control signal such that the corrections are not
     * dominated by the yaw component and such that each motor PWM duty cycle is
     * in [0,1].
     * 
     * @param   controlSignal
     *          control signal to clamp
     * @param   commonThrust
     *          control signal to be sent to the "common motor": this must be in
     *          [0,1]
     * 
     * @return  the clamped attitude control signal.
     */
    AttitudeControlSignal
    clampControlSignal(AttitudeControlSignal controlSignal,
                       real_t commonThrust);

  public:
    /**
     * Update the attitude observer with the given IMU measurement. This
     * function should be called at 238 Hz when the IMU receives a new
     * measurement. Because the attitude control system is implemented with a
     * Kalman filter, this function should be called after AttitudeController::
     * updateControlSignal() is called in order to determine the state estimate
     * for the next cycle.
     * 
     * @param   measurement
     *          new measurement from the IMU
     */
    void updateObserver(AttitudeMeasurement measurement);

    /**
     * Update the attitude controller with the given reference orientation. This
     * function should be called at 238 Hz when the IMU receives a new
     * measurement.
     * 
     * @param   reference
     *          reference orientation to track
     * @param   commonThrust
     *          control signal sent to the "common motor"
     *
     * @return  the control signal to be sent to the "torque motors" until the
     *          next IMU measurement.
     */
    AttitudeControlSignal updateControlSignal(AttitudeReference reference,
                                              real_t commonThrust);

    /**
     * Reset the attitude controller.
     */
    void init();
};
