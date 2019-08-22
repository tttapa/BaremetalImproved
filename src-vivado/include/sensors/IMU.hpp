#pragma once
#include <cstdint>

/* Includes from src. */
#include <LoggerStructs.hpp>  ///< IMUMeasurement

/* Raw 16-bit signed data from the accelerometer. */
struct RawAccelMeasurement {
    RawAccelMeasurement(int16_t axInt, int16_t ayInt, int16_t azInt)
        : axInt{axInt}, ayInt{ayInt}, azInt{azInt} {}
    RawAccelMeasurement() = default;
    int16_t axInt;  ///< Raw 16-bit reading of the acceleration along the x-axis.
    int16_t ayInt;  ///< Raw 16-bit reading of the acceleration along the y-axis.
    int16_t azInt;  ///< Raw 16-bit reading of the acceleration along the z-axis.
};

/* Raw 16-bit signed data from the gyroscope. */
struct RawGyroMeasurement {
    RawGyroMeasurement(int16_t gxInt, int16_t gyInt, int16_t gzInt)
        : gxInt{gxInt}, gyInt{gyInt}, gzInt{gzInt} {}
    RawGyroMeasurement() = default;
    int16_t gxInt;  ///< Raw 16-bit reading of the angular vel. about the x-axis.
    int16_t gyInt;  ///< Raw 16-bit reading of the angular vel. about the y-axis.
    int16_t gzInt;  ///< Raw 16-bit reading of the angular vel. about the z-axis.
};

/**
 * Execute one step of the IMU calibration. If this step is one of the first
 * INVALID_SAMPLES steps, then the data will be ignored. Otherwise the current
 * gyroscope and accelerometer measurements will be added to the running sum. If
 * this step is the last step in the calibration, then the bias will be
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
 * Set up the IMU interrupt system.
 *
 * @return	true
 * 			If setup was successful.
 * @return	false
 * 			Otherwise.
 */
bool initIMUInterruptSystem();

/**
 * Read the gyroscope measurement (rad/s) and acceleration measurement (g) from
 * the IMU.
 * 
 * @return	Most recent IMU measurement.
 */
IMUMeasurement readIMU();
