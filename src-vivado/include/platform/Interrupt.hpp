#pragma once

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
 * 			Otherwise.
 */
bool initIMUInterruptSystem();
