#include <sensors/AHRS.hpp>

/* Includes from src-vivado. */
#include "MadgwickFilter.hpp"

/* Includes from Xilinx. */
#include <xil_io.h>

/** Orientation of the drone, represented by EulerAngles. */
static EulerAngles orientationEuler;

/** Orientation of the drone, updated by Madgwick's algorithm. */
static Quaternion orientation;

EulerAngles getAHRSOrientationEuler() { return orientationEuler; }

Quaternion getAHRSOrientationQuat() { return orientation; }

Quaternion getAHRSJumpedOrientation(float yawJumpToAdd) {
    EulerAngles jumpedOrientationEuler = {orientationEuler.yaw + yawJumpToAdd,
                                          orientationEuler.pitch,
                                          orientationEuler.roll};
    return EulerAngles::eul2quat(jumpedOrientationEuler);
}

void initAHRS(IMUMeasurement imu) {
    /* Use accelerometer values to ensure that the initial quaternion is
	   oriented correctly. */
    orientation = Quaternion::fromDirection(imu.accel);

    /* AHRS is now initialized. */
    xil_printf("AHRS init ok\r\n");
}

void resetAHRSOrientation() { orientation = {}; }

Quaternion updateAHRS(IMUMeasurement imu) {

    /* Calculate next orientation using Madgwick. */
    orientation = MadgwickAHRSUpdate(orientation, imu);

    /* Convert it to EulerAngles. */
    orientationEuler = EulerAngles(orientation);

    /* Return the orientation. */
    return orientation;
}
