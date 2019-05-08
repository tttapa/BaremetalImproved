#include <Motors.hpp>
#include "../../../main/src/PrivateHardwareConstants.hpp"

/**
 * Address of PWM period for the front-left (0), front-right (1) and
 * back-left (2) motors.
 */
const int PERIOD_012_ADDR = XPAR_PWM_AXI_TRIPLE_0_0;

/**
 * Address of PWM period for the back-right (3) motor. This has a different
 * address because a different GPIO block is used for the back-right motor.
 */
const int PERIOD_3_ADDR = XPAR_PWM_AXI_TRIPLE_3_0;

/** Address of PWM duty cycle for the front-left (0) motor. */
const int DUTY_CYCLE_0_ADDR = XPAR_PWM_AXI_TRIPLE_0_0 + 0x04;

/** Address of PWM duty cycle for the front-left (0) motor. */
const int DUTY_CYCLE_1_ADDR = XPAR_PWM_AXI_TRIPLE_0_0 + 0x08;

/** Address of PWM duty cycle for the front-left (0) motor. */
const int DUTY_CYCLE_2_ADDR = XPAR_PWM_AXI_TRIPLE_0_0 + 0x0C;

/** Address of PWM duty cycle for the front-left (0) motor. */
const int DUTY_CYCLE_3_ADDR = XPAR_PWM_AXI_TRIPLE_3_0 + 0x04;

/** Lowest PWM duty cycle sent to the ESCs. */
const float MIN_DUTY_CYCLE = 45.0;

/** Highest PWM duty cycle sent to the ESCs. */
const float MAX_DUTY_CYCLE = 92.0;

/** Dead PWM signal is always off. */
const float DEAD_DUTY_CYCLE = 0.0;

/** PWM frequency sent to the ESCs. */
const float PWM_FREQUENCY = 500.0;

void outputMotorPWM(float v0, float v1, float v2, float v3) {

    /* Clamp duty cycles. */
    if (v0 < MIN_DUTY_CYCLE)
        v0 = MIN_DUTY_CYCLE;
    if (v0 > MAX_DUTY_CYCLE)
        v0 = MAX_DUTY_CYCLE;
    if (v1 < MIN_DUTY_CYCLE)
        v1 = MIN_DUTY_CYCLE;
    if (v1 > MAX_DUTY_CYCLE)
        v1 = MAX_DUTY_CYCLE;
    if (v2 < MIN_DUTY_CYCLE)
        v2 = MIN_DUTY_CYCLE;
    if (v2 > MAX_DUTY_CYCLE)
        v2 = MAX_DUTY_CYCLE;
    if (v3 < MIN_DUTY_CYCLE)
        v3 = MIN_DUTY_CYCLE;
    if (v3 > MAX_DUTY_CYCLE)
        v3 = MAX_DUTY_CYCLE;

    /* Write the period and value to create the PWM signal. Period =
       clock frequency / PWM frequency. Duty cycle = % * period. */
    *PERIOD_012_ADDR   = CLOCK_FREQUENCY / PWM_FREQUENCY;
    *PERIOD_3_ADDR     = CLOCK_FREQUENCY / PWM_FREQUENCY;
    *DUTY_CYCLE_0_ADDR = v0 * *PERIOD_012_ADDR;
    *DUTY_CYCLE_1_ADDR = v1 * *PERIOD_012_ADDR;
    *DUTY_CYCLE_2_ADDR = v2 * *PERIOD_012_ADDR;
    *DUTY_CYCLE_3_ADDR = v3 * *PERIOD_3_ADDR;
}