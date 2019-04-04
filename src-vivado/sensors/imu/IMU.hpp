// Original: BareMetal/src/IMU/IMU.h
// Original: BareMetal/src/IMU/LSM9DS1_Registers.h


/* Raw 16-bit signed data from the accelerometer. */
struct RawAccelMeasurement {
    int axInt;
    int ayInt;
    int azInt;
}


/* Acceleration measurement in g. */
struct AccelMeasurement {
    float ax;
    float ay;
    float az;
}


/* Raw 16-bit signed data from the gyroscope. */
struct RawGyroMeasurement {
    int gxInt;
    int gyInt;
    int gzInt;
}


/* Angular velocity measurement in rad/s. */
struct GyroMeasurement {
    float gx;
    float gy;
    float gz;
}


/* Measured IMU angular velocity (gx,gy,gz) and acceleration (ax,ay,az). */
struct IMUMeasurement {
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
};


/**
 * Execute one step of the IMU calibration. If this step is one of the first
 * `IMU::INVALID_SAMPLES` steps, then the data will be ignored. Otherwise the
 * current gyroscope and accelerometer measurements will be added to the running
 * sum. If this step is the last step in the calibration, then the bias will be
 * calculated from this sum.
 * 
 * @return	true
 * 			if calibration has reached its final step
 * @return	false
 * 			otherwise
 */
bool calibrateIMUStep();


/**
 * Initialize the IMU. Verify that it is connected, initialize the calibration
 * variables and flush the FIFO.
 * 
 * @return	true
 * 			if initialization was successful
 * @return	false
 * 			otherwise
 */
bool initIMU();


/**
 * Read the gyroscope measurement (rad/s) and acceleration measurement (g) from the IMU.
 * 
 * @return	most recent IMU measurement.
 */
IMUMeasurement readIMU();

