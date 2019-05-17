#pragma once

/* Includes from src. */
#include <LoggerStructs.hpp>  ///< BuzzerInstruction

/**
 * Outputs a PWM signal to the buzzer with the given period and the
 * given volume.
 * 
 * @param   instruction
 *          BuzzerInstruction to send to the buzzer. // TODO: in [0,1] ipv int
 */
void outputBuzzerPWM(BuzzerInstruction instruction);

/**
 * Outputs a PWM signal to the buzzer with the given period and the
 * given volume.
 * 
 * @param   duration
 *          Duration of the BuzzerInstruction to send to the buzzer.
 * @param   period
 *          Period of the BuzzerInstruction to send to the buzzer.
 * @param   volume
 *          Volume of the BuzzerInstruction to send to the buzzer.
 */
void outputBuzzerPWM(float duration, int period, int volume);