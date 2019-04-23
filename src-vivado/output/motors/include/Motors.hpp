/**
 * Clamps the given duty cycles if necessary and sends them to the PWM generator
 * to create the appropriate PWM waves for each of the motors.
 * 
 * @param   v0
 *          duty cycle of PWM signal to be sent to the front-left motor in [0,1]
 * @param   v1
 *          duty cycle of PWM signal to be sent to the front-right motor in [0,1]
 * @param   v2
 *          duty cycle of PWM signal to be sent to the back-left motor in [0,1]
 * @param   v3
 *          duty cycle of PWM signal to be sent to the back-right motor in [0,1]
 */
void outputMotorPWM(float v0, float v1, float v2, float v3);