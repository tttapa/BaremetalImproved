#include "PrivateHardwareConstants.hpp" /* Registers & channels */
#include <AxiGpio.hpp>                  /* Header file */
#include <xgpio.h>                      /* AXI GPIO functions */
#include <xparameters.h>                /* Project parameters */
#include <xscugic.h>                    /* Xilinx functions */

/* Port used for LEDs in LED GPIO. */
const int LED_CHANNEL = 1;

/* Port used for heartbeat in LED GPIO. */
const int HEARTBEAT_CHANNEL = 2;

/* Port used for testpin in TESTPIN GPIO. */
const int TESTPIN_CHANNEL = 1;

/* Address of GPIO that LEDs are connected to. */
const int GPIO_DEVICE_LED = XPAR_AXI_GPIO_LED_DEVICE_ID;

/* Address of GPIO that the TESTPINs are connected to. */
const int GPIO_DEVICE_TESTPIN = XPAR_AXI_GPIO_TESTPINS_DEVICE_ID;

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

void writeValueToTestPin(bool value) {
    XGpio_DiscreteWrite(&axi_gpio_2, TESTPIN_CHANNEL, value);
}

void writeToLEDs(LEDInstruction values) {
    int value =
        values.led1 + 2 * values.led2 + 4 * values.led3 + 8 * values.led4;
    XGpio_DiscreteWrite(&axi_gpio_1, LED_CHANNEL, value);
}
