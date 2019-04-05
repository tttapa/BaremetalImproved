// Original: BareMetal/src/AHRS/ahrs.c
/*******************************************************************************
 *  Attitude and heading reference system source code
 *  This module takes IMU readings and tracks the orientation of the quadcopter.
 *  There is no magnetometer, so some drift around the Z axis is to be expected.
 *  This file should NEVER be changed by the students
 *  Author: p. coppens
********************************************************************************/
#include "../include/AHRS.hpp"
#include "../src/MadgwickFilter.hpp"
#include <xil_io.h>


/* Orientation of the drone, updated by Madgwick's algorithm. */
static Quaternion orientation;


void initAHRS(IMUMeasurement imu) {

	// TODO: use new quaternion functions instead of Quat32, Vec32, ...
	orientation = Quaternion::unit();

	/*
	 * use accelerometer values to ensure that the initial quaternion is oriented correctly
	 */
	ColVector<3> accel = {imu.ax, imu.ay, imu.az};
	accel = normalize(accel);

	/*
	 * calculate angle (based on direction of acceleration
	 * theoretically we need arcsin here, but we know that angle is relatively small so the error is acceptable
	 */
	ColVector<3> angle = { std::asin(accel[1][0]), std::asin(-(real_t)accel[0][0]), 0};

	/*
	 * update orientation (initial error is the actual measured orientation after all
	 */
	Quaternion rot = Quaternion::quatFromVec(angle);
	orientation = (orientation + rot).normalize();

	/*
	 * AHRS is now initialized
	 */
	xil_printf("AHRS init ok\r\n");
}


Quaternion updateAHRS(IMUMeasurement imu) {

    orientation = MadgwickAHRSUpdate(orientation, imu);
	return orientation;

}
