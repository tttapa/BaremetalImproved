#pragma once

/* Includes from src. */
#include <EulerAngles.hpp>
#include <Quaternion.hpp>
#include <LoggerStructs.hpp>  ///< IMUMeasurement

/* Includes from src-vivado. */
#include <sensors/IMU.hpp>

// TODO: comments
EulerAngles getAHRSOrientationEuler();
Quaternion getAHRSOrientationQuat();
void resetAHRSOrientation();

/**
 * Initialize the Attitude and Heading Reference System using the initial IMU measurement.
 */
void initAHRS(IMUMeasurement imu);

// TODO: comments
void setYaw(float yawRads);

/**
 * Update the Attitude and Heading Reference System using the new IMU measurement.
 * 
 * @param   imu
 *          New IMU measurement.
 *
 * @return  New orientation estimate as EulerAngles.
 */
EulerAngles updateAHRS(IMUMeasurement imu);
