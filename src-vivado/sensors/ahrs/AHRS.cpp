// Original: BareMetal/src/AHRS/ahrs.c
/*******************************************************************************
 *  Attitude and heading reference system source code
 *  This module takes IMU readings and tracks the orientation of the quadcopter.
 *  There is no magnetometer, so some drift around the Z axis is to be expected.
 *  This file should NEVER be changed by the students
 *  Author: p. coppens
********************************************************************************/
#include "AHRS.hpp"

// Given 
void AHRS::initAHRS() {

	/* Reset orientation. */
	orientation = Quaternion();

	Quaternion rot, temp;
	orientation.w = 1;
	orientation.x = 0;
	orientation.y = 0;
	orientation.z = 0;

	/*
	 * use accelerometer values to ensure that the initial quaternion is oriented correctly
	 */
	readAcc();
	Vec32 accel = {ax, ay, az};
	vec32_normalize_32f16(&accel, &accel);

	/*
	 * calculate angle (based on direction of acceleration
	 * theoretically we need arcsin here, but we know that angle is relatively small so the error is acceptable
	 */
	Vec32 angle = {  asin(accel.y),
	                 asin(-accel.x),
	                 0};

	/*
	 * update orientation (initial error is the actual measured orientation after all
	 */
	quat32_from_vector(&rot, &angle);           /* q_IMU = build a quaternion from angle vector */
	quat32_multiply(&temp, &orientation, &rot); /* q_error = calculate quat error and assign to temp */
	quat32_normalize(&orientation, &temp);      /* assign normalised temp to IMU */

	/*
	 * AHRS is now initialized
	 */
	ahrs=1;
	xil_printf("AHRS init ok\r\n");
}

// Given, replaced by Madgwick
Quat32 updateAHRS(IMUMeasurement imu) {

    orientation = MadgwickAHRSupdateIMU(orientation, imu);
	return orientation;

}
