#include <sensors/IMU.hpp>

/* Includes from src. */
#include <Quaternion.hpp>

/* Includes from src-vivado. */
#include "../platform/IIC.hpp"
#include "LSM9DS1_Registers.h"
#include <platform/AxiGpio.hpp>
#include <platform/Interrupt.hpp>

/* Includes from Xilinx. */
#include <sleep.h>  // TODO: is usleep() necessary?
#include <xil_io.h>
#include <xparameters.h>

#include "IMUTypes.hpp"
#include "LSM9DS1Def.hpp"
#include <PublicHardwareConstants.hpp>
/* Gyroscope and accelerometer frequencies are set to 238 Hz. */
const IMUFrequency IMU_FREQUENCY = IMUFrequency::FREQ_238_HZ;

/* Maximum measurable angular velocity is 2000 deg/s. */
const GyroMaxSpeed GYRO_MAX_SPEED = GyroMaxSpeed::SPEED_2000_DPS;

/* Maximum measurable acceleration in 16 g. */
const AccelMaxSpeed ACCEL_MAX_SPEED = AccelMaxSpeed::SPEED_16_G;

/* Amount of samples to take to determine bias. */
const int CALIBRATION_SAMPLES = 512;

/* Amount of samples to remove at the start of calibration. */
const int INVALID_SAMPLES = 16;

/** Bias of the gyroscope, set on last step of calibration. */
GyroMeasurement gyroBias;

/** Bias of the accelerometer as a quaternion, set on last step of calibration. */
Quaternion accelBiasQuat;

/** Norm of the accelerometer quaternion bias. */
float accelBiasNorm;

/** Sum of the raw gyroscope measurements, used to calculate bias. */
long gyroRawSum[3];

/** Sum of the raw accelerometer measurements, used to calculate bias. */
long accelRawSum[3];

/** Number of calibration steps taken. */
int calibrationStepCounter;

const float PI = 3.14159265358979323846;

/**
 * Convert the raw gyroscope reading to rad/s.
 * 
 * @param	rawGyro
 * 			Signed 16-bit gyroscope measurement.
 * @return	The raw gyroscope measurement converted to rad/s.
 */
float calcGyro(int rawGyro) {

    /** Calculate the resolution of the gyroscope in degree/s. */
    static constexpr float gyroResolution =
        getIMUValue(GYRO_MAX_SPEED) / 32768.0; /* 2^15 */

    /** Convert the raw gyro value to rad/s. */
    return gyroResolution * (PI / 180.0) * (float) rawGyro;
}

/**
 * Convert the raw accelerometer reading to g.
 * 
 * @param	rawAccel
 * 			Signed 16-bit accelerometer measurement.
 * @return	The raw accelerometer measurement converted to g.
 */
float calcAccel(int rawAccel) {

    /** Calculate the resolution of the accelerometer in g. */
    static constexpr float accelResolution =
        getIMUValue(ACCEL_MAX_SPEED) / 32768.0; /* 2^15 */

    /** Convert the raw accel value to g. */
    return accelResolution * (float) rawAccel;
}

/**
 * Read six uint8s from the gyroscope and construct the three raw measurements
 * for the gyroscope (signed 16-bit).
 * 
 * @return	Raw gyroscope measurement.
 */
RawGyroMeasurement readGyro() {

    /* Read data from gyroscope.*/
    static uint8_t gyroData[GYRO_DATA_SIZE];
    iicReadReg(gyroData, OUT_X_L_G, LSM9DS1_GX_ADDR, GYRO_DATA_SIZE);

    /* Raw 16-bit signed data from readings. */
    int16_t gxInt = (gyroData[1] << 8) + gyroData[0];
    int16_t gyInt = (gyroData[3] << 8) + gyroData[2];
    int16_t gzInt = (gyroData[5] << 8) + gyroData[4];

    /* Return raw measurement. */
    return RawGyroMeasurement{gxInt, gyInt, gzInt};
}

/**
 * Convert raw gyroscope measurement to rad/s and remove bias.
 * 
 * @param	raw
 * 			Raw gyroscope measurement.
 * @param	bias
 * 			Bias of the gyroscope, calculated during calibration of IMU.
 * @return	Unbiased gyroscope measurement in rad/s.
 */
GyroMeasurement getGyroMeasurement(RawGyroMeasurement raw,
                                   GyroMeasurement bias) {

    /* Gyroscope measurements with bias removed in rad/s. */
    float gx = -(calcGyro(raw.gxInt) - bias.gx);
    float gy = +(calcGyro(raw.gyInt) - bias.gy);
    float gz = -(calcGyro(raw.gzInt) - bias.gz);

    /* Return measurement. */
    return GyroMeasurement{gx, gy, gz};
}

/**
 * Read six uint8s from the accelerometer and construct the three raw
 * measurements for the accelerometer (signed 16-bit).
 * 
 * @return	Raw accelerometer measurement.
 */
RawAccelMeasurement readAccel() {

    /* Read data from accelerometer. */
    static u8 accelData[ACCEL_DATA_SIZE];
    iicReadReg(accelData, OUT_X_L_XL, LSM9DS1_GX_ADDR, ACCEL_DATA_SIZE);

    /* Raw 16-bit signed data from readings. */
    int16_t axInt = (accelData[1] << 8) + accelData[0];
    int16_t ayInt = (accelData[3] << 8) + accelData[2];
    int16_t azInt = (accelData[5] << 8) + accelData[4];

    /* Return raw measurement. */
    return RawAccelMeasurement{axInt, ayInt, azInt};
}

/**
 * Convert raw accelerometer measurement to g and remove bias.
 * 
 * @param	raw
 * 			Raw accelerometer measurement.
 * @param	bias
 * 			Bias of the accelerometer, calculated during calibration of IMU.
 * @return	unbiased accelerometer measurement in g.
 * @todo	Test this function.
 */
ColVector<3> getAccelMeasurement(RawAccelMeasurement raw, Quaternion biasQuat,
                                 float biasNorm) {
    /* Accelerometer measurements with bias removed in g. */
    ColVector<3> correctedAccel = (biasQuat).rotate(ColVector<3>{
        -calcAccel(raw.axInt),  // TODO: check signs
        +calcAccel(raw.ayInt),
        -calcAccel(raw.azInt),
    });
    correctedAccel /= biasNorm;
    return correctedAccel;
}

bool calibrateIMUStep() {

    /* Log start of calibration. */
    if (calibrationStepCounter == 0)
        xil_printf("calibrating IMU, reading %i samples \r\n",
                   CALIBRATION_SAMPLES);

    /* Have the LEDs blink: slow down by factor 2^4, cycle through 8 states. */
    int ledIndex = (calibrationStepCounter >> 4) % 8;
    int ledValue = 0x0;
    ledValue += (ledIndex >= 1 ? 0x1 : 0);
    ledValue += (ledIndex >= 2 && ledIndex <= 6 ? 0x2 : 0);
    ledValue += (ledIndex >= 3 && ledIndex <= 5 ? 0x4 : 0);
    ledValue += (ledIndex == 4 ? 0x8 : 0);
    writeToLEDs(ledIndex >= 1, ledIndex >= 2 && ledIndex <= 6,
                ledIndex >= 3 && ledIndex <= 5, ledIndex == 4);

    /* Increment counter. */
    calibrationStepCounter++;

    /* Read raw sensor data. */
    RawGyroMeasurement rawGyro = readGyro();
    usleep(100);  // TODO: sleep to prevent corruption?
    RawAccelMeasurement rawAccel = readAccel();

    /* First measurementes contain invalid parameters. */
    if (calibrationStepCounter > INVALID_SAMPLES) {
        gyroRawSum[0] += rawGyro.gxInt;
        gyroRawSum[1] += rawGyro.gyInt;
        gyroRawSum[2] += rawGyro.gzInt;
        accelRawSum[0] += rawAccel.axInt;
        accelRawSum[1] += rawAccel.ayInt;
        accelRawSum[2] += rawAccel.azInt;
    }

    /* Final calibration step reached. */
    if (calibrationStepCounter == CALIBRATION_SAMPLES + INVALID_SAMPLES) {

        float factor = 1.0 / (float) (CALIBRATION_SAMPLES);

        /* Calculate gyroscope bias. */
        gyroBias.gx = calcGyro(gyroRawSum[0] * factor);
        gyroBias.gy = calcGyro(gyroRawSum[1] * factor);
        gyroBias.gz = calcGyro(gyroRawSum[2] * factor);

        /* Calculate accelerometer bias quaternion. */
        ColVector<3> accelBiasAverage = {
            -calcAccel(accelRawSum[0] * factor),
            +calcAccel(accelRawSum[1] * factor),
            -calcAccel(accelRawSum[2] * factor),
        };
        accelBiasQuat = Quaternion::fromDirection(accelBiasAverage);
        accelBiasNorm = norm(accelBiasAverage);

        /* Turn off all LEDs. */
        writeToLEDs(0, 0, 0, 0);
        xil_printf("calibrated IMU \r\n");

        /* Initialization successful. */
        return true;
    }

    /* Initialization still busy. */
    return false;
}

bool initIMU() {

    /* Set the global tick frequency/period. */
    TICKS_PER_SECOND = getIMUValue(IMU_FREQUENCY);
    SECONDS_PER_TICK = 1.0 / TICKS_PER_SECOND;

    /* Temporary array to store initialization readings. */
    u8 temp[1];

    xil_printf("init IMU started \r\n");

    /* Check WHO_AM_I to see if the IMU is connected. */
    iicReadReg(temp, WHO_AM_I_XG, LSM9DS1_GX_ADDR, 1);
    usleep(100);
    xil_printf("received: 0x%02x\r\n", *temp);
    if (*temp != WHO_AM_I_XG_ID) {
        xil_printf("WHO_AM_I FAILED\r\n");
        return false;
    }
    iicReadReg(temp, WHO_AM_I_M, LSM9DS1_M_ADDR, 1);
    usleep(100);
    xil_printf("received: 0x%02x\r\n", *temp);
    if (*temp != WHO_AM_I_M_ID) {
        xil_printf("WHO_AM_I FAILED\r\n");
        return false;
    }
    xil_printf("WHO_AM_I CORRECT\r\n");

    /* See https://www.st.com/resource/en/datasheet/DM00103319.pdf */
    /* ... Hz gyro output data rate (ODR), max ... degrees/s,  ... Hz cutoff. */
    iicWriteToReg(CTRL_REG1_G,
                  getIMUBits(IMU_FREQUENCY) + getIMUBits(GYRO_MAX_SPEED),
                  LSM9DS1_GX_ADDR);

    /* Low-power mode disabled, high-pass filter disabled, 15 Hz cutoff. */
    iicWriteToReg(CTRL_REG3_G, 0b00000000, LSM9DS1_GX_ADDR);

    /* Positive signs for pitch (x), roll(y) and yaw (z), standard
	   orientation. */
    iicWriteToReg(ORIENT_CFG_G, 0b00000000, LSM9DS1_GX_ADDR);

    /* Enable pitch, roll, yaw output, interrupt request latched, interrupt
	   generator uses 6D for position recognition. */
    iicWriteToReg(CTRL_REG4, 0b00111010, LSM9DS1_GX_ADDR);

    /* No data decimation, enable ax, ay, az output. */
    iicWriteToReg(CTRL_REG5_XL, 0b00111000, LSM9DS1_GX_ADDR);

    /* ... Hz accel output data rate (ODR), max ... g, ... Hz bandwith. */
    iicWriteToReg(CTRL_REG6_XL,
                  getIMUBits(IMU_FREQUENCY) + getIMUBits(ACCEL_MAX_SPEED),
                  LSM9DS1_GX_ADDR);

    /* Accel high-resolution mode disabled, (Ts/50) Hz low-pass cutoff
	   frequency (bypassed), high-pass filter bypassed for interrupt. */
    iicWriteToReg(CTRL_REG7_XL, 0b00000000, LSM9DS1_GX_ADDR);

    /* Continuous FIFO mode: if FIFO is full, the new sample overwrites the
	   older samples, flag=1 if FIFO contains at least 2 elements (unused). */
    iicWriteToReg(FIFO_CTRL, 0b11000010, LSM9DS1_GX_ADDR);

    /* Initialize calibration variables. */
    calibrationStepCounter = 0;
    gyroBias               = {};
    accelBiasQuat          = Quaternion::unit();
    accelBiasNorm          = 1;
    for (int i = 0; i < 3; i++) {
        gyroRawSum[i]  = 0;
        accelRawSum[i] = 0;
    }

    /* Clear data in IMU's FIFO. */
    for (int i = 0; i < 5; i++) {
        readAccel();
        readGyro();
        usleep(100);  // TODO: sleep to prevent corruption?
    }

    /* IMU is initiated. */
    // TODO: beep_initiated() here
    //beep_initiated();
    xil_printf("IMU initiated\r\n");
    xil_printf("Starting interrupts\r\n");

    /* Enable interrupts when accelerometer has a measurement. */
    iicWriteToReg(INT1_CTRL, 0x01, LSM9DS1_GX_ADDR);

    /* Initialization successful. */
    return true;
}

IMUMeasurement readIMU() {

    /* Read gyroscope and convert to rad/s. */
    RawGyroMeasurement gyroRaw = readGyro();
    GyroMeasurement gyro       = getGyroMeasurement(gyroRaw, gyroBias);
    usleep(100);  // TODO: sleep to prevent corruption?

    /* Read accelerometer and convert to g. */
    RawAccelMeasurement accelRaw = readAccel();
    ColVector<3> accel =
        getAccelMeasurement(accelRaw, accelBiasQuat, accelBiasNorm);

    /* Return IMU measurement (gyro+accel). */
    return IMUMeasurement{gyro.gx,  gyro.gy,  gyro.gz,
                          accel[0], accel[1], accel[2]};
}
