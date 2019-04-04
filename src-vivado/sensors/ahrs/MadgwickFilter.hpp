// Original: BareMetal/src/utils/MadgwickAHRS.h
#include "quaternion.h"


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
 * Apply Madgwick's algorithm using all available measurement data (gyroscope,
 * accelerometer, magnetometer) and the given orientation.
 * 
 * @param   orientation
 *          last orientation of the drone
 * @param   imu
 *          current measurement of the IMU (gyro+accel+mag)
 * 
 * @return updated drone orientation, according to Madgwick's algorithm.
 */
Quat32 MadgwickAHRSUpdateFull(Quat32 orientation, FullIMUMeasurement imu);


/**
 * Apply Madgwick's algorithm using only the measurement data from the gyroscope
 * and the accelerometer (and the given orientation).
 * 
 * @param   orientation
 *          last orientation of the drone
 * @param   imu
 *          current measurement of the IMU (gyro+accel)
 * 
 * @return updated drone orientation, according to Madgwick's algorithm.
 */
Quat32 MadgwickAHRSUpdate(Quat32 orientation, IMUMeasurement imu);
