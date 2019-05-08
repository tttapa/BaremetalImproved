/*******************************************************************************
 *   Integrated Integrated Circuit device driver header file
 *   This file contains all functions required to use the IIC.
 *   This file should NEVER be changed by the students.
 *   Author: w. devries
 ******************************************************************************/
#include <xil_types.h>

/**
 * Write to one of the registers of the IMU.
 *
 * @param   register_addr
 *          Register we want to write to.
 * @param   u8data
 *          8 bits of data to write.
 * @param   deviceAddr
 *          Address to write to (LSM9DS1_GX_ADDR for Gyr/Acc, or LSM9DS1_M_ADDR
 *          for Magnetometer).
 */
void iicWriteToReg(u8 register_addr, u8 u8Data, u16 deviceAddr);

/**
 * Read from one of the registers of the IMU.
 *
 * @param   recv_buffer
 *          Pointer to buffer where info is saved.
 * @param   register_addr
 *          Register we want to read from.
 * @param   deviceAddr
 *          Address to write to (LSM9DS1_GX_ADDR for Gyr/Acc, or LSM9DS1_M_ADDR
 *          for Magnetometer).
 * @param   size
 *          Amount of bytes to read.
 */
void iicReadReg(u8 *recv_buffer, u8 register_addr, u16 deviceAddr, int size);
