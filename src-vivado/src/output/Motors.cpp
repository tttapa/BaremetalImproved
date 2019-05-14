#include <output/Motors.hpp>

/* Includes from src-vivado. */
#include "../PrivateHardwareConstants.hpp"

#pragma region Constants
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
#pragma endregion

void outputMotorPWM(MotorSignals motorSignals) {

    /* We don't have to clamp the given signals to [0,1] because MotorSignals
       does this automatically. Scale the motor signals to duty cycle limits. */
    float v0, v1, v2, v3;
    v0 = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * motorSignals.v0;
    v1 = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * motorSignals.v1;
    v2 = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * motorSignals.v2;
    v3 = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * motorSignals.v3;

    /* Write the period and value to create the PWM signal. Period =
       clock frequency / PWM frequency. Duty cycle = % * period. */
    *PERIOD_012_ADDR   = CLOCK_FREQUENCY / PWM_FREQUENCY;
    *PERIOD_3_ADDR     = CLOCK_FREQUENCY / PWM_FREQUENCY;
    *DUTY_CYCLE_0_ADDR = v0 * *PERIOD_012_ADDR;
    *DUTY_CYCLE_1_ADDR = v1 * *PERIOD_012_ADDR;
    *DUTY_CYCLE_2_ADDR = v2 * *PERIOD_012_ADDR;
    *DUTY_CYCLE_3_ADDR = v3 * *PERIOD_3_ADDR;
}

void outputMotorPWM(float v0, float v1, float v2, float v3) {
    outputMotorPWM({v0, v1, v2, v3});
}