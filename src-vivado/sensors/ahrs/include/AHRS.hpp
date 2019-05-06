// Original: BareMetal/src/AHRS/ahrs.h
/**********************************************************************************************************************
 *   Attitude and heading reference system header file
 *  This module takes IMU readings and tracks the orientation of the quadcopter.
 *  There is no magnetometer, so some drift around the Z axis is to be expected.
 *  This file should NEVER be changed by the students
 *   Author: p. coppens
***********************************************************************************************************************/
#include "../../imu/include/IMU.hpp"
#include <Quaternion.hpp>
#include <EulerAngles.hpp>
#pragma once

/**
 * Add the given yaw to the Euler representation of the AHRS's orientation, then
 * return the quaternion representation of that orientation.
 * 
 * @param   yawJumpToAdd
 *          Radians to add to the Euler representation of the orientation.
 * 
 * @return  The quaternion representation of the orientation rotated by the
 *          given yaw "jump".
 */
Quaternion getJumpedOrientation(float yawJumpToAdd) {

    /**
 * Initialize the Attitude and Heading Reference System using the initial IMU measurement.
 */
    void initAHRS(IMUMeasurement imu);

    /**
 * Update the Attitude and Heading Reference System using the new IMU measurement.
 * 
 * @param   imu
 *          New IMU measurement.
 */
    Quaternion updateAHRS(IMUMeasurement imu);
