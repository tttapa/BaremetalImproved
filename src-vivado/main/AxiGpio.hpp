// TODO: should the files be commented like this?
// TODO: see https://developer.lsst.io/v/DM-5063/docs/cpp_docs.html
/**
 * @file ExampleClass.cc
 *
 * @brief This message displayed in Doxygen Files index
 *
 * @ingroup PackageName
 * (Note: this needs exactly one @defgroup somewhere)
 *
 * @author Joe Smith
 * Contact: js@lsst.org
 *
 */

/**
 * Initialise AXI GPIO pins.
 */
bool initAxiGpio();


/**
 * Generate a heartbeat by writing 0, then 1 to the heartbeat channel.
 */
void generateHeartbeat();


/**
 * Write 1 to the test pin to probe the length of an interrupt.
 */
void testpinHigh();


/**
 * Write the given value to the LEDs on the Zybo. The value should be in [0x0000, 0x1111], and each bit will be written to a different LED.
 * 
 * @param   value
 *          the value to write to the LEDs
 */
void writeValueToLEDs(int value);
