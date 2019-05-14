#pragma once

/* Includes from src. */
#include <EulerAngles.hpp>
#include <Quaternion.hpp>
#include <SensorTypes.hpp>  ///< IMUMeasurement

/* Includes from src-vivado. */
#include <sensors/IMU.hpp>

// TODO: comments
EulerAngles getAHRSOrientationEuler();
Quaternion getAHRSOrientationQuat();
void resetAHRSOrientation();

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
Quaternion getAHRSJumpedOrientation(float yawJumpToAdd);

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
