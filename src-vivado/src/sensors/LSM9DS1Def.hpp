
/**
 * Number of u8s used to construct the 3 raw measurements (signed 16-bit) for
 * the gyroscope.
 */
const int GYRO_DATA_SIZE = 6;

/**
 * Number of u8s used to construct the 3 raw measurements (signed 16-bit) for
 * the accelerometer.
 */
const int ACCEL_DATA_SIZE = 6;

/** Slave address for the gyroscope + accelerometer. */
const int LSM9DS1_GX_ADDR = 0x6B;

/** Slave address for the magnetometer. */
const int LSM9DS1_M_ADDR = 0x1E;

/** Interrupt address for the IMU. */
const int INT1_CTRL = 0x0C;

/** "Who am I" address for the gyroscope + accelerometer. */
const int WHO_AM_I_XG = 0x0F;

/** "Who am I" ID for the gyroscope + accelerometer. */
const int WHO_AM_I_XG_ID = 0x68;

/** "Who am I" address for the magnetometer. */
const int WHO_AM_I_M = 0x0F;

/** "Who am I" ID for the magnetometer. */
const int WHO_AM_I_M_ID = 0x3D;

/** Gyroscope control register 1. */
const int CTRL_REG1_G = 0x10;

/** Gyroscope control register 3. */
const int CTRL_REG3_G = 0x12;

/** Gyroscope sign and orientation register. */
const int ORIENT_CFG_G = 0x13;

/** Gyroscope data output register. */
const int OUT_X_L_G = 0x18;

/** Control register 4. */
const int CTRL_REG4 = 0x1E;

/** Accelerometer control register 5. */
const int CTRL_REG5_XL = 0x1F;

/** Accelerometer control register 6. */
const int CTRL_REG6_XL = 0x20;

/** Accelerometer control register 7. */
const int CTRL_REG7_XL = 0x21;

/** Accelerometer data output register. */
const int OUT_X_L_XL = 0x28;

/** FIFO control register. */
const int FIFO_CTRL = 0x2E;
