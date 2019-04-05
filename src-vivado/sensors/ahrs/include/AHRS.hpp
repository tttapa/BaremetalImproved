// Original: BareMetal/src/AHRS/ahrs.h
/**********************************************************************************************************************
 *   Attitude and heading reference system header file
 *  This module takes IMU readings and tracks the orientation of the quadcopter.
 *  There is no magnetometer, so some drift around the Z axis is to be expected.
 *  This file should NEVER be changed by the students
 *   Author: p. coppens
***********************************************************************************************************************/
#include <Quaternion.hpp>
#include "../../imu/include/IMU.hpp"
#pragma once


/**
 * Initialize the Attitude and Heading Reference System using the initial IMU measurement.
 */
void initAHRS(IMUMeasurement imu);


/**
 * Update the Attitude and Heading Reference System using the new IMU measurement.
 * 
 * @param   imu
 *          new IMU measurement
 */
Quaternion updateAHRS(IMUMeasurement imu);
