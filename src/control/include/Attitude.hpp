#include <Quaternion.hpp>

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
     * Attitude quaternion reference to track. Naturally the equilibrium point
     * will have an angular velocity of zero and no torque motor angular
     * velocity.
     */
    AttitudeReference reference;

    /**
     * Most recent measurement from the IMU, consisting of one quaternion for
     * the drone's orientation and three floats for the drone's angular
     * velocity, measured in rad/s.
     */
    AttitudeMeasurement measurement;

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

    void updateAttitudeKFEstimate(AttitudeState, AttitudeControlSignal,
                                  AttitudeMeasurement, int);

    void getAttitudeControllerOutput(AttitudeState, AttitudeReference,
                                     AttitudeControlSignal,
                                     AttitudeIntegralWindup, int, real_t);

  public:
    /**
     * Try updating the attitude observer at 238 Hz. Because the attitude
     * control system is directly coupled to the IMU measurements, which updates
     * at 238Hz, the attitude estimate will change every time this function is
     * called. 
     */
    void updateObserver(AttitudeMeasurement);

    /**
     * Try updating the attitude controller at 238 Hz. Because the attitude
     * control system is directly coupled to the IMU measurements, which updates
     * at 238Hz, the attitude control signal will change every time this function
     * is called.
     */
    AttitudeControlSignal updateControlSignal();

    void initializeController();
    
    void idleController();
};
