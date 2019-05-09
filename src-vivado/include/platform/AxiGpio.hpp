#pragma once

/* Includes from src. */
#include <OutputTypes.hpp>  ///< LEDInstruction

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
