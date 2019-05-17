#pragma once

/* Includes from src. */
#include <LoggerStructs.hpp>  ///< LEDInstruction

/**
 * Initialise AXI GPIO pins.
 */
bool initAxiGpio();

/**
 * Generate a heartbeat by writing 0, then 1 to the heartbeat channel. This is
 * used to activate the kill switch when the software hangs.
 */
void generateHeartbeat();

/**
 * Write the given value to the test pin to probe the length of an interrupt.
 * 
 * @param   value
 *          The value to write to the test pin (on/off).
 */
void writeValueToTestPin(bool value);

/**
 * Write the given values to the LEDs on the Zybo. The values should be
 * represented by a LEDInstruction.
 * 
 * @param   values
 *          The LEDInstruction to write to the LEDs.
 */
void writeToLEDs(LEDInstruction values);

/**
 * Write the given values to the LEDs on the Zybo. The values should be
 * represented by a LEDInstruction.
 * 
 * @param   led1
 *          Whether the first led should be lit.
 * @param   led2
 *          Whether the second led should be lit.
 * @param   led3
 *          Whether the third led should be lit.
 * @param   led4
 *          Whether the fourth led should be lit.
 */
void writeToLEDs(bool led1, bool led2, bool led3, bool led4);
