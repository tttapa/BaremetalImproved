// Original: BareMetal/src/intc/intc.h
#pragma once

/*******************************************************************************
 *   Interrupt control header file
 *   This file contains all interrupt related methods and variables.
 *   This file should NEVER be changed by the students.
 *   Author: w. devries
 ******************************************************************************/

/**
 * Set up the interrupt system (master, IIC, IMU).
 *
 * @return  true 
 *          if setup was successful 
 * @return  false
 *          otherwise
 */
bool initInterrupt();


/*******************************************************************************
 *   Integrated Integrated Circuit device driver header file
 *   This file contains all functions required to use the IIC.
 *   This file should NEVER be changed by the students.
 *   Author: w. devries
 ******************************************************************************/

/**
 * Write to one of the registers of the IMU.
 *
 * @param   register_addr
 *          Register we want to write to
 * @param   u8data
 *          8 bits of data to write
 * @param   device
 *          Gyr/Acc if 1, Magnetometer if 0
 */
void iicWriteToReg(u8 register_addr, u8 u8Data, int device);

/**
 * Read from one of the registers of the IMU.
 *
 * @param   recv_buffer
 *          pointer to buffer where info is saved
 * @param   register_addr
 *          register we want to read from
 * @param   device
 *          Gyr/Acc if 1, Magnetometer if 0
 * @param   size
 *          amount of bytes to read
 */
void iicReadReg(u8 *recv_buffer, u8 register_addr, int device, int size);
