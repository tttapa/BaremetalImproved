#pragma once

/* Includes from src. */
#include <SensorTypes.hpp>  ///< IMUMeasurement

/* Raw 16-bit signed data from the accelerometer. */
struct RawAccelMeasurement {
    RawAccelMeasurement(int axInt, int ayInt, int azInt)
        : axInt{axInt}, ayInt{ayInt}, azInt{azInt} {}
    RawAccelMeasurement() = default;
    int axInt;  ///< Raw 16-bit reading of the acceleration along the x-axis.
    int ayInt;  ///< Raw 16-bit reading of the acceleration along the y-axis.
    int azInt;  ///< Raw 16-bit reading of the acceleration along the z-axis.
};

/* Raw 16-bit signed data from the gyroscope. */
struct RawGyroMeasurement {
    RawGyroMeasurement(int gxInt, int gyInt, int gzInt)
        : gxInt{gxInt}, gyInt{gyInt}, gzInt{gzInt} {}
    RawGyroMeasurement() = default;
    int gxInt;  ///< Raw 16-bit reading of the angular vel. about the x-axis.
    int gyInt;  ///< Raw 16-bit reading of the angular vel. about the y-axis.
    int gzInt;  ///< Raw 16-bit reading of the angular vel. about the z-axis.
};

/**
 * Execute one step of the IMU calibration. If this step is one of the first
 * `INVALID_SAMPLES` steps, then the data will be ignored. Otherwise the
 * current gyroscope and accelerometer measurements will be added to the running
 * sum. If this step is the last step in the calibration, then the bias will be
 * calculated from this sum.
 * 
 * @return	true
 * 			If calibration has reached its final step.
 * @return	false
 * 			Otherwise.
 */
bool calibrateIMUStep();

/**
 * Gets the hardware address of the device.
 * 
 * @param   device
 *          
 */

/**
 * Initialize the IMU. Verify that it is connected, initialize the calibration
 * variables and flush the FIFO.
 * 
 * @return	true
 * 			If initialization was successful.
 * @return	false
 * 			Otherwise.
 */
bool initIMU();

/**
 * Read the gyroscope measurement (rad/s) and acceleration measurement (g) from
 * the IMU.
 * 
 * @return	Most recent IMU measurement.
 */
IMUMeasurement readIMU();
