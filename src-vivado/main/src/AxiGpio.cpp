#include "AxiGpio.hpp"				/* Header file */
#include "HardwareConstants.hpp"	/* Registers & channels */
#include "xgpio.h"					/* AXI GPIO functions */
#include "xscugic.h"				/* Xilinx functions */
#include "xparameters.h"			/* Project parameters */


// TODO: is this signature correct? These should act as "globals" within this cpp file.
/* GPIO Device driver instance for LEDs and heartbeat. */
XGpio axi_gpio_1;

/*  GPIO Device driver instance for testpin. */
XGpio axi_gpio_2;


bool initAxiGpio() {

    int status;

	/* Initialise AXI GPIO used for LEDs and heartbeat. */
	status = XGpio_Initialize(&axi_gpio_1, AXI_GPIO::GPIO_DEVICE_LED);
	if (status != XST_SUCCESS) {
		xil_printf("GPIO init to the LEDs failed!\r\n");
		return false;
	}

	/* Initialise AXI GPIO used to measure the interrupt-looplength. */
	status = XGpio_Initialize(&axi_gpio_2, AXI_GPIO::GPIO_DEVICE_TESTPIN);
	if (status != XST_SUCCESS) {
		xil_printf("GPIO init to the testpin failed!\r\n");
		return false;
	}

	/* Set the direction for the channels to output. */
	XGpio_SetDataDirection(&axi_gpio_1, AXI_GPIO::LED_CHANNEL, 0x00);
	XGpio_SetDataDirection(&axi_gpio_1, AXI_GPIO::HEARTBEAT_CHANNEL, 0x00);
	XGpio_SetDataDirection(&axi_gpio_2, AXI_GPIO::TESTPIN_CHANNEL, 0x00);

	/* Start heartbeat. */
	XGpio_DiscreteWrite(&axi_gpio_1, AXI_GPIO::HEARTBEAT_CHANNEL, 1);

    /* Setup successful. */
    xil_printf("AXI GPIO initialization successful.");
    return true;
}


void generateHeartbeat() {
    XGpio_DiscreteWrite(&axi_gpio_1, AXI_GPIO::HEARTBEAT_CHANNEL, 0);
    XGpio_DiscreteWrite(&axi_gpio_1, AXI_GPIO::HEARTBEAT_CHANNEL, 1);
}


void writeValueToTestPin(bool value) {
    XGpio_DiscreteWrite(&axi_gpio_2, AXI_GPIO::TESTPIN_CHANNEL, value);
}


void writeValueToLEDs(int value) {
    XGpio_DiscreteWrite(&axi_gpio_1, AXI_GPIO::LED_CHANNEL, value);
}
