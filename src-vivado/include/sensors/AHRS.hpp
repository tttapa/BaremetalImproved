#pragma once

/* Includes from src. */
#include <Quaternion.hpp>
#include <LoggerStructs.hpp>  ///< IMUMeasurement

/* Includes from src-vivado. */
#include <sensors/IMU.hpp>

/** Reset the AHRS's orientation to the identity quaternion. */
void resetAHRSOrientation();

/**
 * Initialize the Attitude and Heading Reference System (AHRS) using the
 * initial IMU measurement.
 */
void initAHRS(IMUMeasurement imu);

/**
 * Update the Attitude and Heading Reference System using the new IMU measurement.
 * 
 * @param   imu
 *          New IMU measurement.
 * 
 * @return  The update AHRS orientation measurement.
 */
Quaternion updateAHRS(IMUMeasurement imu);
