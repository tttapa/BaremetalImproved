#include <sensors/AHRS.hpp>

/* Includes from src-vivado. */
#include "MadgwickFilter.hpp"

/* Includes from Xilinx. */
#include <xil_io.h>

/** Orientation of the drone, updated by Madgwick's algorithm. */
static Quaternion orientation;

void initAHRS(IMUMeasurement imu) {
    /* Use accelerometer values to ensure that the initial quaternion is
	   oriented correctly. */
    orientation = Quaternion::fromDirection(imu.accel.a);

    /* AHRS is now initialized. */
    xil_printf("AHRS init ok\r\n");
}

void resetAHRSOrientation() { orientation = {}; }

Quaternion updateAHRS(IMUMeasurement imu) {

    /* Calculate next orientation using Madgwick. */
    orientation = MadgwickAHRSUpdate(orientation, imu);

    /* Return the orientation. */
    return orientation;
}