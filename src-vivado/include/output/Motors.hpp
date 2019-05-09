#pragma once

/* Includes from src. */
#include <OutputTypes.hpp>  ///< MotorSignals

/**
 * Clamps the given duty cycles if necessary and sends them to the PWM generator
 * to create the appropriate PWM waves for each of the motors.
 * 
 * @param   motorSignals
 *          MotorSignals to send to the four ESCs (front-left, front-right,
 *          back-left, back-right, each of which in [0,1]).
 */
void outputMotorPWM(MotorSignals motorSignals);