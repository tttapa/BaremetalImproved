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
	/*
	 * use accelerometer values to ensure that the initial quaternion is oriented correctly
	 */
	ColVector<3> accel = {imu.ax, imu.ay, imu.az};
	
	orientation = Quaternion::fromDirection(accel);

	/*
	 * AHRS is now initialized
	 */
	xil_printf("AHRS init ok\r\n");
}


Quaternion updateAHRS(IMUMeasurement imu) {

    orientation = MadgwickAHRSUpdate(orientation, imu);
	return orientation;

}
