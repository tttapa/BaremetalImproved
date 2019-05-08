#pragma once

/**
 * Output a PWM signal at 500 Hz with the given duty cycle to the Wireless Power
 * Transfer (WPT) team.
 * 
 * @param   dutyCycle
 *          Duty cycle to be sent to the WPT team.
 */
void outputWPT(float dutyCycle);