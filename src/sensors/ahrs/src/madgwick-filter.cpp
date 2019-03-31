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
#include "MadgwickAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	238.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

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

Quat32 MadgwickAHRSUpdateFull(Quat32 orientation, FullIMUMeasurement imu) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((imu.mx == 0.0f) && (imu.my == 0.0f) && (imu.mz == 0.0f)) {
		MadgwickAHRSupdateIMU(orientation, IMUMeasurement {	imu.gx, imu.gy, imu.gz, 
														   	imu.ax, imu.ay, imu.az });
		return;
	}

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

		// Normalise magnetometer measurement
		recipNorm = invSqrt(imu.mx * imu.mx + imu.my * imu.my + imu.mz * imu.mz);
		imu.mx *= recipNorm;
		imu.my *= recipNorm;
		imu.mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * orientation.w * imu.mx;
		_2q0my = 2.0f * orientation.w * imu.my;
		_2q0mz = 2.0f * orientation.w * imu.mz;
		_2q1mx = 2.0f * orientation.x * imu.mx;
		_2q0 = 2.0f * orientation.w;
		_2q1 = 2.0f * orientation.x;
		_2q2 = 2.0f * orientation.y;
		_2q3 = 2.0f * orientation.z;
		_2q0q2 = 2.0f * orientation.w * orientation.y;
		_2q2q3 = 2.0f * orientation.y * orientation.z;
		q0q0 = orientation.w * orientation.w;
		q0q1 = orientation.w * orientation.x;
		q0q2 = orientation.w * orientation.y;
		q0q3 = orientation.w * orientation.z;
		q1q1 = orientation.x * orientation.x;
		q1q2 = orientation.x * orientation.y;
		q1q3 = orientation.x * orientation.z;
		q2q2 = orientation.y * orientation.y;
		q2q3 = orientation.y * orientation.z;
		q3q3 = orientation.z * orientation.z;

		// Reference direction of Earth's magnetic field
		hx = imu.mx * q0q0 - _2q0my * orientation.z + _2q0mz * orientation.y + imu.mx * q1q1 + _2q1 * imu.my * orientation.y + _2q1 * imu.mz * orientation.z - imu.mx * q2q2 - imu.mx * q3q3;
		hy = _2q0mx * orientation.z + imu.my * q0q0 - _2q0mz * orientation.x + _2q1mx * orientation.y - imu.my * q1q1 + imu.my * q2q2 + _2q2 * imu.mz * orientation.z - imu.my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * orientation.y + _2q0my * orientation.x + imu.mz * q0q0 + _2q1mx * orientation.z - imu.mz * q1q1 + _2q2 * imu.my * orientation.z - imu.mz * q2q2 + imu.mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - imu.ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - imu.ay) - _2bz * orientation.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - imu.mx) + (-_2bx * orientation.z + _2bz * orientation.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - imu.my) + _2bx * orientation.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - imu.mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - imu.ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - imu.ay) - 4.0f * orientation.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - imu.az) + _2bz * orientation.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - imu.mx) + (_2bx * orientation.y + _2bz * orientation.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - imu.my) + (_2bx * orientation.z - _4bz * orientation.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - imu.mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - imu.ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - imu.ay) - 4.0f * orientation.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - imu.az) + (-_4bx * orientation.y - _2bz * orientation.w) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - imu.mx) + (_2bx * orientation.x + _2bz * orientation.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - imu.my) + (_2bx * orientation.w - _4bz * orientation.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - imu.mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - imu.ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - imu.ay) + (-_4bx * orientation.z + _2bz * orientation.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - imu.mx) + (-_2bx * orientation.w + _2bz * orientation.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - imu.imu.my) + _2bx * orientation.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - imu.mz);
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
// IMU algorithm update

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


int instability_fix = 1;

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	if (instability_fix == 0)
	{
		/* original code */
		float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		return y;
	}
	else if (instability_fix == 1)
	{
		/* close-to-optimal  method with low cost from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
		unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
		float tmp = *(float*)&i;
		return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
	}
	else
	{
		/* optimal but expensive method: */
		return 1.0f / sqrtf(x);
	}
}

//====================================================================================================
// END OF CODE
//====================================================================================================
