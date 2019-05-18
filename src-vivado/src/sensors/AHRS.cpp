#include <sensors/AHRS.hpp>

/* Includes from src-vivado. */
#include "MadgwickFilter.hpp"

/* Includes from Xilinx. */
#include <xil_io.h>

/** Orientation of the drone, updated by Madgwick's algorithm. */
static Quaternion orientation;

EulerAngles getAHRSOrientationEuler() { return EulerAngles::quat2eul(orientationEuler) }; }

Quaternion getAHRSOrientationQuat() { return orientation; }

void initAHRS(IMUMeasurement imu) {
    /* Use accelerometer values to ensure that the initial quaternion is
	   oriented correctly. */
    orientation = Quaternion::fromDirection(imu.accel.a);

    /* AHRS is now initialized. */
    xil_printf("AHRS init ok\r\n");
}

void resetAHRSOrientation() { orientation = {}; }

void setYaw(float yawRads) {
    EulerAngles eul = EulerAngles::quat2eul(orientation);
    eul.yaw = yawRads;
    orientation = EulerAngles::eul2quat(eul);
}

EulerAngles updateAHRS(IMUMeasurement imu) {

    /* Calculate next orientation using Madgwick. */
    orientation = MadgwickAHRSUpdate(orientation, imu);

    /* Return the orientation. */
    return EulerAngles::quat2eul(orientation);
}
