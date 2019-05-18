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

/**
 * Multiply the AHRS's orientation by the given difference quaternion (on the
 * left side). This function is used to keep the attitude controller's
 * orientation estimate near the identity quaternion.
 * 
 * @param   diffQuat
 *          The quaternion that multiplied on the left side of the attitude
 *          controller's orientation estimate in order to keep it near the
 *          identity quaternion.
 * 
 * @return  The jumped AHRS orientation measurement.
 */
Quaternion updateAHRSDiffQuat(Quaternion diffQuat);
