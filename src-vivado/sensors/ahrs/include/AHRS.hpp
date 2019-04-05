// Original: BareMetal/src/AHRS/ahrs.h
/**********************************************************************************************************************
 *   Attitude and heading reference system header file
 *  This module takes IMU readings and tracks the orientation of the quadcopter.
 *  There is no magnetometer, so some drift around the Z axis is to be expected.
 *  This file should NEVER be changed by the students
 *   Author: p. coppens
***********************************************************************************************************************/
#include "MadgwickFilter.hpp"
#include "quaternion.hpp"   // TODO: use new Quaternion
#pragma once

/**
 * Get the Attitude and Heading Reference System's orientation.
 */
Quat32 getOrientation();


/**
 * Initialize the Attitude and Heading Reference System using the initial IMU measurement.
 */
void init(IMUMeasurement imu);


/**
 * Update the Attitude and Heading Reference System using the new IMU measurement.
 * 
 * @param   imu
 *          new IMU measurement
 */
void update(IMUMeasurement imu) {
    MadgwickAHRSupdateIMU(orientation, imu);
}