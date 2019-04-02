#include "xparameters.h"

// TODO: should these be "static const int" in implementation file?
/* Address of GPIO that LEDs are connected to. */
#define GPIO_DEVICE_LED  	(XPAR_AXI_GPIO_LED_DEVICE_ID)	

/* Address of GPIO that the TESTPINs are connected to. */
#define GPIO_DEVICE_TESTPIN (XPAR_AXI_GPIO_TESTPINS_DEVICE_ID)

/* Port used for LEDs in LED GPIO. */
#define LED_CHANNEL 		1

/* Port used for heartbeat in LED GPIO. */
#define HEARTBEAT_CHANNEL 	2

/* Port used for testpin in TESTPIN GPIO. */
#define TESTPIN_CHANNEL 	1


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
