#include <Quaternion.hpp>
#include <Globals.h>
#include <configuration.hpp>

/**
 * Attitude reference to track, consisting of a single quaternion.
 */
struct AttitudeReference {
    Quaternion q = Quaternion(1, 0, 0, 0);  // Orientation
};

/**
 * Measurement from the IMU, consisting of one quaternion for the drone's
 * orientation and three floats for the drone's angular velocity, measured in
 * rad/s.
 */
struct AttitudeMeasurement {
    Quaternion q;
    real_t wx;  // X angular velocity (rad/s)
    real_t wy;  // Y angular velocity (rad/s)
    real_t wz;  // Z angular velocity (rad/s)
};

/**
 * Estimate of the state of the drone's attitude, consisting of the drone's
 * orientation (1 quaternion), angular velocity in rad/s (3 components: wx, wy,
 * wz) and the angular velocity of the torque motors in rad/s (3 components: nx,
 * ny, nz).
 */
struct AttitudeState {
    Quaternion q = Quaternion(1, 0, 0, 0);  // Orientation
    real_t wx;                              // X angular velocity (rad/s)
    real_t wy;                              // Y angular velocity (rad/s)
    real_t wz;                              // Z angular velocity (rad/s)
    real_t nx;                              // X motor angular velocity (rad/s)
    real_t ny;                              // Y motor angular velocity (rad/s)
    real_t nz;                              // Z motor angular velocity (rad/s)
};

/**
 * Integral of the error of the quaternion components q1, q2 and q3.
 */
struct AttitudeIntegralWindup {
    real_t q1;
    real_t q2;
    real_t q3;
};

/**
 * PWM control signals sent to the torque motors (3 components: ux, uy, uz).
 */
struct AttitudeControlSignal {
    real_t ux;  // X motor signal (/)
    real_t uy;  // Y motor signal (/)
    real_t uz;  // Z motor signal (/)
};

/**
 * Four floats representing the duty cycles to be sent to the four motors
 * (front-left, front-right, back-left, back-right). The four values should
 * be in [0, 1].
 */
struct MotorDutyCycles {
  real_t v0;  /* Front-left motor duty cycle. */
  real_t v1;  /* Front-right motor duty cycle. */
  real_t v2;  /* Back-left motor duty cycle. */
  real_t v3;  /* Back-right motor duty cycle. */
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
MotorDutyCycles transformAttitudeControlSignal(AttitudeControlSignal controlSignal, real_t commonThrust);


/**
 * Class to control the attitude of the drone. The first part is an observer to
 * estimate the drone's orientation, angular velocity and the angular velocity
 * of the "torque motors". Next, there is a controller to send appropriate PWM
 * signals to the torque motors based on how far the drone's state estimate
 * deviates from the reference state.
 *
 * To achieve this, the AttitudeController contains variables to store the
 * reference orientation, state estimate, integral windup and control signal.
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
     * Maximum signal value (clamp) for the z-torque motor. This prevents the
     * drone from trying to turn too fast to correct the yaw.
     */
    real_t uzClamp;

    // TODO: clamp where?
    void clampAttitudeControllerOutput(AttitudeControlSignal, real_t);

    /**
     * Update the given attitude estimate using the code generator.
     * 
     * @param   stateEstimate
     *          estimate of the current attitude state, determined last cycle
     * @param   controlSignal
     *          current control signal
     * @param   measurement
     *          current measurement from the IMU
     * @param   droneConfiguration
     *          configuration of the drone
     */
    void updateObserverCodegen(AttitudeState stateEstimate,
                               AttitudeControlSignal controlSignal,
                               AttitudeMeasurement measurement,
                               int droneConfiguration);

    /**
     * Update the given attitude control signal using the code generator.
     * 
     * @param   stateEstimate
     *          estimate of the current attitude state, determined last cycle
     * @param   reference
     *          height reference to track
     * @param   controlSignal
     *          control signal to update
     * @param   integralWindup
     *          integral windup to update
     * @param   droneConfiguration
     *          configuration of the drone
     */
    void updateControlSignalCodegen(AttitudeState stateEstimate,
                                    AttitudeReference reference,
                                    AttitudeControlSignal controlSignal,
                                    AttitudeIntegralWindup integralWindup,
                                    int droneConfiguration);


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
     *          the reference orientation to track
     *
     * @return  the control signal to be sent to the "torque motors".
     */
    AttitudeControlSignal updateControlSignal(AttitudeReference reference);

    void initializeController();
    
    void idleController();
};
