#pragma once

/* Includes from src. */
#include <LoggerStructs.hpp>  ///< MotorSignals

/**
 * Clamps the given duty cycles if necessary and sends them to the PWM generator
 * to create the appropriate PWM waves for each of the motors.
 * 
 * @param   motorSignals
 *          MotorSignals to send to the four ESCs (front-left, front-right,
 *          back-left, back-right, each of which in [0,1]).
 */
void outputMotorPWM(MotorSignals motorSignals);

/**
 * Clamps the given duty cycles if necessary and sends them to the PWM generator
 * to create the appropriate PWM waves for each of the motors.
 * 
 * @param   v0
 *          Signal to send to the front-left motor in [0,1].
 * @param   v1
 *          Signal to send to the front-right motor in [0,1].
 * @param   v2
 *          Signal to send to the back-left motor in [0,1].
 * @param   v3
 *          Signal to send to the back-right motor in [0,1].
 */
void outputMotorPWM(float v0, float v1, float v2, float v3);