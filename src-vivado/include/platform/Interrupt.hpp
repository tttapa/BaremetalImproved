// Original: BareMetal/src/intc/intc.h
#pragma once

/*******************************************************************************
 *   Interrupt control header file
 *   This file contains all interrupt related methods and variables.
 *   This file should NEVER be changed by the students.
 *   Author: w. devries
 ******************************************************************************/

/**
 * Set up the interrupt system (master, IIC).
 *
 * @return  true 
 *          If setup was successful.
 * @return  false
 *          Otherwise.
 */
bool initInterrupt();

/**
 * Set up the IMU interrupt system.
 *
 * @return	true
 * 			If setup was successful.
 * @return	false
 * 			Otherwise;
 */
bool initIMUInterruptSystem();
