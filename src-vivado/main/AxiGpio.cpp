#include "AxiGpio.hpp"
#include "xgpio.h"
#include "xscugic.h"


/* GPIO Device driver instance for LEDs and heartbeat. */
static XGpio axi_gpio_1;

/*  GPIO Device driver instance for testpin. */
static XGpio axi_gpio_2;


bool initAxiGpio() {

    int status;

	/* Initialise AXI GPIO used for LEDs and heartbeat. */
	status = XGpio_Initialize(&axi_gpio_1, GPIO_DEVICE_LED);
	if (status != XST_SUCCESS) {
		xil_printf("GPIO init to the LEDs failed!\r\n");
		return false;
	}

	/* Initialise AXI GPIO used to measure the interrupt-looplength. */
	status = XGpio_Initialize(&axi_gpio_2, GPIO_DEVICE_TESTPIN);
	if (status != XST_SUCCESS) {
		xil_printf("GPIO init to the testpin failed!\r\n");
		return false;
	}

	/* Set the direction for the channels to output. */
	XGpio_SetDataDirection(&axi_gpio_1, LED_CHANNEL, 0x00);
	XGpio_SetDataDirection(&axi_gpio_1, HEARTBEAT_CHANNEL, 0x00);
	XGpio_SetDataDirection(&axi_gpio_2, TESTPIN_CHANNEL, 0x00);

	/* Start heartbeat. */
	XGpio_DiscreteWrite(&axi_gpio_1, HEARTBEAT_CHANNEL, 1);

    /* Setup successful. */
    xil_printf("AXI GPIO initialization successful.");
    return true;
}


void generateHeartbeat() {
    XGpio_DiscreteWrite(&axi_gpio_1, HEARTBEAT_CHANNEL, 0);
    XGpio_DiscreteWrite(&axi_gpio_1, HEARTBEAT_CHANNEL, 1);
}


void testpinHigh() {
    XGpio_DiscreteWrite(&axi_gpio_2, 1, 0x1);
}


void writeValueToLEDs(int value) {
    XGpio_DiscreteWrite(&axi_gpio_1, LED_CHANNEL, value);
}
