// Original: BareMetal/src/AHRS/ahrs.c
/*******************************************************************************
 *  Attitude and heading reference system source code
 *  This module takes IMU readings and tracks the orientation of the quadcopter.
 *  There is no magnetometer, so some drift around the Z axis is to be expected.
 *  This file should NEVER be changed by the students
 *  Author: p. coppens
********************************************************************************/
#include "MadgwickFilter.hpp"
#include <AHRS.hpp>
#include <SensorTypes.hpp>
#include <xil_io.h>

/** Orientation of the drone, represented by EulerAngles. */
static EulerAngles orientationEuler;

/** Orientation of the drone, updated by Madgwick's algorithm. */
static Quaternion orientation;

Quaternion getJumpedOrientation(float yawJumpToAdd) {
    EulerAngles jumpedOrientationEuler = {orientationEuler.yaw + yawJumpToAdd,
                                          orientationEuler.pitch,
                                          orientationEuler.roll};
    return EulerAngles::eul2quat(jumpedOrientationEuler);
}

void initAHRS(IMUMeasurement imu) {
    /* Use accelerometer values to ensure that the initial quaternion is
	   oriented correctly. */
    ColVector<3> accel = {imu.ax, imu.ay, imu.az};

    orientation = Quaternion::fromDirection(accel);

    /* AHRS is now initialized. */
    xil_printf("AHRS init ok\r\n");
}

Quaternion updateAHRS(IMUMeasurement imu) {

    /* Calculate next orientation using Madgwick. */
    orientation = MadgwickAHRSUpdate(orientation, imu);

    /* Convert it to EulerAngles. */
    orientationEuler = EulerAngles(orientation);

    /* Return the orientation. */
    return orientation;
}
