#include <output/Motors.hpp>

/* Includes from src-vivado. */
#include "../PrivateHardwareConstants.hpp"

/**
 * Address of PWM period for the front-left (0), front-right (1) and
 * back-left (2) motors.
 */
uint32_t *const PERIOD_012_ADDR = (uint32_t *) XPAR_PWM_AXI_TRIPLE_0_0;

/**
 * Address of PWM period for the back-right (3) motor. This has a different
 * address because a different GPIO block is used for the back-right motor.
 */
uint32_t *const PERIOD_3_ADDR = (uint32_t *) XPAR_PWM_AXI_TRIPLE_3_0;

/** Address of PWM duty cycle for the front-left (0) motor. */
uint32_t *const DUTY_CYCLE_0_ADDR = ((uint32_t *) XPAR_PWM_AXI_TRIPLE_0_0) + 1;

/** Address of PWM duty cycle for the front-left (0) motor. */
uint32_t *const DUTY_CYCLE_1_ADDR = ((uint32_t *) XPAR_PWM_AXI_TRIPLE_0_0) + 2;

/** Address of PWM duty cycle for the front-left (0) motor. */
uint32_t *const DUTY_CYCLE_2_ADDR = ((uint32_t *) XPAR_PWM_AXI_TRIPLE_0_0) + 3;

/** Address of PWM duty cycle for the front-left (0) motor. */
uint32_t *const DUTY_CYCLE_3_ADDR = ((uint32_t *) XPAR_PWM_AXI_TRIPLE_3_0) + 1;

/** Lowest PWM duty cycle sent to the ESCs. */
const float MIN_DUTY_CYCLE = 0.45;

/** Highest PWM duty cycle sent to the ESCs. */
const float MAX_DUTY_CYCLE = 0.92;

/** Dead PWM signal is always off. */
const float DEAD_DUTY_CYCLE = 0.0;

/** PWM frequency sent to the ESCs. */
const float PWM_FREQUENCY = 500.0;

void outputMotorPWM(MotorSignals motorSignals) {

    /* Scale motor signals to duty cycle limits. */
    float v0, v1, v2, v3;
    v0 = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * motorSignals.v0;
    v1 = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * motorSignals.v1;
    v2 = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * motorSignals.v2;
    v3 = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * motorSignals.v3;

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
