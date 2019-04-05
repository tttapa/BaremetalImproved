// Original: BareMetal/src/utils/MadgwickAHRS.c
// Original: BareMetal/src/utils/MadgwickAHRS.h
//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files
#include "MadgwickFilter.hpp"


//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	238.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain
#define INSTABILITY_FIX 1		// Use numerical instability fix

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

Quat32 MadgwickAHRSUpdate(Quat32 orientation, IMUMeasurement imu) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-orientation.x * imu.gx - orientation.y * imu.gy - orientation.z * imu.gz);
	qDot2 = 0.5f * (orientation.w * imu.gx + orientation.y * imu.gz - orientation.z * imu.gy);
	qDot3 = 0.5f * (orientation.w * imu.gy - orientation.x * imu.gz + orientation.z * imu.gx);
	qDot4 = 0.5f * (orientation.w * imu.gz + orientation.x * imu.gy - orientation.y * imu.gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((imu.ax == 0.0f) && (imu.ay == 0.0f) && (imu.az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
		imu.ax *= recipNorm;
		imu.ay *= recipNorm;
		imu.az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * orientation.w;
		_2q1 = 2.0f * orientation.x;
		_2q2 = 2.0f * orientation.y;
		_2q3 = 2.0f * orientation.z;
		_4q0 = 4.0f * orientation.w;
		_4q1 = 4.0f * orientation.x;
		_4q2 = 4.0f * orientation.y;
		_8q1 = 8.0f * orientation.x;
		_8q2 = 8.0f * orientation.y;
		q0q0 = orientation.w * orientation.w;
		q1q1 = orientation.x * orientation.x;
		q2q2 = orientation.y * orientation.y;
		q3q3 = orientation.z * orientation.z;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * imu.ax + _4q0 * q1q1 - _2q1 * imu.ay;
		s1 = _4q1 * q3q3 - _2q3 * imu.ax + 4.0f * q0q0 * orientation.x - _2q0 * imu.ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * imu.az;
		s2 = 4.0f * q0q0 * orientation.y + _2q0 * imu.ax + _4q2 * q3q3 - _2q3 * imu.ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * imu.az;
		s3 = 4.0f * q1q1 * orientation.z - _2q1 * imu.ax + 4.0f * q2q2 * orientation.z - _2q2 * imu.imu.ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;

	}

	// Integrate rate of change of quaternion to yield quaternion
	orientation.w += qDot1 * (1.0f / sampleFreq);
	orientation.x += qDot2 * (1.0f / sampleFreq);
	orientation.y += qDot3 * (1.0f / sampleFreq);
	orientation.z += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(orientation.w * orientation.w + orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z);
	orientation.w *= recipNorm;
	orientation.x *= recipNorm;
	orientation.y *= recipNorm;
	orientation.z *= recipNorm;
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {

#if INSTABILITY_FIX
	/* close-to-optimal  method with low cost from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
	unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
	float tmp = *(float*)&i;
	return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
#else
	/* original code */
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
#endif

}

//====================================================================================================
// END OF CODE
//====================================================================================================
