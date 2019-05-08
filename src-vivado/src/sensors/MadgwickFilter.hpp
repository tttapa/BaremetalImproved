#pragma once
#include <Quaternion.hpp>
#include <SensorTypes.hpp>

//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================


/* Algorithm gain (used in gradient descent). */
extern volatile float beta;


/**
 * Apply Madgwick's algorithm using only the measurement data from the gyroscope
 * and the accelerometer (and the given orientation).
 * 
 * @param   orientation
 *          Last orientation of the drone
 * @param   imu
 *          Current measurement of the IMU (gyro+accel)
 * 
 * @return  The updated drone orientation, according to Madgwick's algorithm.
 */
Quaternion MadgwickAHRSUpdate(Quaternion orientation, IMUMeasurement imu);
