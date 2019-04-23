#include <real_t.h>
#include <Attitude.hpp>

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
}

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
 * Send the given duty cycles to the PWM generator to create the appropriate
 * PWM waves for each of the motors.
 */
void outputPWM(MotorDutyCycles dutyCycles);
