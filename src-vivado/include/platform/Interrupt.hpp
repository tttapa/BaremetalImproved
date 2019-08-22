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
