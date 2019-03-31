// Original: BareMetal/src/utils/MadgwickAHRS.h
//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <main.h>
#include "quaternion.h"

//----------------------------------------------------------------------------------------------------
// Variable declaration
extern int instability_fix;
extern volatile float beta;				// algorithm gain

//---------------------------------------------------------------------------------------------------
// Function declarations

Quat32 MadgwickAHRSUpdateFull(Quat32 orientation, FullIMUMeasurement imu);  // Gyrometer on, Accelerometer on, Magnetometer on
Quat32 MadgwickAHRSUpdate(Quat32 orientation, IMUMeasurement imu);          // Gyrometer on, Accelerometer on, Magnetometer off

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
