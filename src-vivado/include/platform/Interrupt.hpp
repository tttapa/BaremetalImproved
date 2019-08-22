#pragma once

#include <xscugic.h>

/**
 * Set up the interrupt system (master, IIC).
 *
 * @return  true 
 *          If setup was successful.
 * @return  false
 *          Otherwise.
 */
bool initInterrupt();

extern XScuGic InterruptController;

/* Interrupt controller device ID. */
const int INTC_DEVICE_ID = XPAR_PS7_SCUGIC_0_DEVICE_ID;
