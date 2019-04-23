#include "../include/Motors.hpp"
#include "../../../main/src/HardwareConstants.hpp"

void outputMotorPWM(float v0, float v1, float v2, float v3) {

    /* Clamp duty cycles. */
    if(v0 < MOTORS::MIN_DUTY_CYCLE) v0 = MOTORS::MIN_DUTY_CYCLE;
    if(v0 > MOTORS::MAX_DUTY_CYCLE) v0 = MOTORS::MAX_DUTY_CYCLE;
    if(v1 < MOTORS::MIN_DUTY_CYCLE) v1 = MOTORS::MIN_DUTY_CYCLE;
    if(v1 > MOTORS::MAX_DUTY_CYCLE) v1 = MOTORS::MAX_DUTY_CYCLE;
    if(v2 < MOTORS::MIN_DUTY_CYCLE) v2 = MOTORS::MIN_DUTY_CYCLE;
    if(v2 > MOTORS::MAX_DUTY_CYCLE) v2 = MOTORS::MAX_DUTY_CYCLE;
    if(v3 < MOTORS::MIN_DUTY_CYCLE) v3 = MOTORS::MIN_DUTY_CYCLE;
    if(v3 > MOTORS::MAX_DUTY_CYCLE) v3 = MOTORS::MAX_DUTY_CYCLE;

    /* Write the period and value to create the PWM signal. */
    *MOTORS::PERIOD_012_ADDR = CLOCK_FREQUENCY / MOTORS::PWM_FREQUENCY; // Clockfreq / PWM freq
    *MOTORS::PERIOD_3_ADDR = CLOCK_FREQUENCY / MOTORS::PWM_FREQUENCY;
    *MOTORS::DUTY_CYCLE_0_ADDR = v0 * *MOTORS::PERIOD_012_ADDR;         // % * period
    *MOTORS::DUTY_CYCLE_1_ADDR = v1 * *MOTORS::PERIOD_012_ADDR;
    *MOTORS::DUTY_CYCLE_2_ADDR = v2 * *MOTORS::PERIOD_012_ADDR;
    *MOTORS::DUTY_CYCLE_3_ADDR = v3 * *MOTORS::PERIOD_3_ADDR;

}