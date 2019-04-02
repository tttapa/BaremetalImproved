// Original: BareMetal/src/AHRS/ahrs.h
/**********************************************************************************************************************
 *   Attitude and heading reference system header file
 *  This module takes IMU readings and tracks the orientation of the quadcopter.
 *  There is no magnetometer, so some drift around the Z axis is to be expected.
 *  This file should NEVER be changed by the students
 *   Author: p. coppens
***********************************************************************************************************************/
#ifndef AHRS_H
#define AHRS_H

// Header Files
// ====================================================================================================================
#include "../comm/iic.h"
#include "../intc/intc.h"
#include "../utils/quaternion.h"
#include "xtime_l.h"

// Prototype definitions
// ====================================================================================================================

/**
 *  ahrs_tick updates the orientation with the current accelerometer and gyroscope readings.
 *  computes quaternions to give attitude and heading reference system
 *  input is previous estimation quaternion and IMU measured quaternion
 * 	output is estimated quaternion
 * 	TIME = �7.5 us or 4900 clock cycles
 */
Quat32 updateAHRS(IMUMeasurement imu);

/**
 * Initialize the attitude and heading reference system
 * Calculate starting orientation quaternion referenced at inertial axes
 */
void initAHRS();

#endif // AHRS_H
