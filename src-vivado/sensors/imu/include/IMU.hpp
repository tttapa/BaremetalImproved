// Original: BareMetal/src/IMU/IMU.h
// Original: BareMetal/src/IMU/LSM9DS1_Registers.h
#pragma once

/* Raw 16-bit signed data from the accelerometer. */
struct RawAccelMeasurement {
    int axInt;  ///< Raw 16-bit reading of the acceleration along the x-axis.
    int ayInt;  ///< Raw 16-bit reading of the acceleration along the y-axis.
    int azInt;  ///< Raw 16-bit reading of the acceleration along the z-axis.
};

/* Acceleration measurement in g. */
struct AccelMeasurement {
    float ax;   ///< Acceleration along the x-axis in g.
    float ay;   ///< Acceleration along the y-axis in g.
    float az;   ///< Acceleration along the z-axis in g.
};

/* Raw 16-bit signed data from the gyroscope. */
struct RawGyroMeasurement {
    int gxInt;  ///< Raw 16-bit reading of the angular vel. about the x-axis.
    int gyInt;  ///< Raw 16-bit reading of the angular vel. about the y-axis.
    int gzInt;  ///< Raw 16-bit reading of the angular vel. about the z-axis.
};

/* Angular velocity measurement in rad/s. */
struct GyroMeasurement {
    float gx;   ///< Angular velocity about the x-axis in rad/s.
    float gy;   ///< Angular velocity about the y-axis in rad/s.
    float gz;   ///< Angular velocity about the z-axis in rad/s.
};


/* Measured IMU angular velocity (gx,gy,gz) and acceleration (ax,ay,az). */
struct IMUMeasurement {
    float gx;   ///< Acceleration along the x-axis in g.
    float gy;   ///< Acceleration along the y-axis in g.
    float gz;   ///< Acceleration along the z-axis in g.
    float ax;   ///< Angular velocity about the x-axis in rad/s.
    float ay;   ///< Angular velocity about the y-axis in rad/s.
    float az;   ///< Angular velocity about the z-axis in rad/s.
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

