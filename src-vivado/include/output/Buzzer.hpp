#pragma once

/* Includes from src. */
#include <OutputTypes.hpp>  ///< BuzzerInstruction

/**
 * Outputs a PWM signal to the buzzer with the given period and the
 * given volume.
 * 
 * @param   instruction
 *          BuzzerInstruction to send to the buzzer. // TODO: in [0,1] ipv int
 */
void outputBuzzerPWM(BuzzerInstruction instruction);