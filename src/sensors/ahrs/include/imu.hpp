// Original: BareMetal/src/IMU/IMU.h
// Original: BareMetal/src/IMU/LSM9DS1_Registers.h

/**********************************************************************************************************************
*   IMU device driver header
*   This file contains all functions required to use the IMU.
*   This file should normally not be changed by the students.
*   Author: w. devries
***********************************************************************************************************************/
#ifndef IMU_H_
#define IMU_H_

// Header Files
// ====================================================================================================================
#include "xtime_l.h"
#include <math.h>

#include "LSM9DS1_Registers.h"
#include "../main.h"

// Constant definitions
// ====================================================================================================================
// Slave address for the IMU
#define LSM9DS1_GX_ADDR			0x6B
#define LSM9DS1_M_ADDR			0x1E

// FIFO Mode Types
#define	FIFO_OFF   0
#define	FIFO_THS   1
#define	FIFO_CONT_TRIGGER   3
#define	FIFO_OFF_TRIGGER   4
#define	FIFO_CONT  5

// IMU States
#define IMU_OFF		0		// The IMU has not been initialized yet
#define IMU_INIT	1		// The IMU has been initialized
#define IMU_CLEAN	2		// The IMU buffer has been cleared
#define IMU_READY	3		// The IMU is ready for taking measurements

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -0.48 // Declination (degrees) in Brussels.


/* Measured IMU angular velocity (gx,gy,gz). */
struct GyroMeasurement {
	float gx;
	float gy;
	float gz;
}

/* Measured IMU angular acceleration (ax,ay,az). */
struct AccMeasurement {
	float ax;
	float ay;
	float az;
}

/* Measured IMU magnetometer (mx,my,mz). */
struct MagMeasurement {
	float mx;
	float my;
	float mz;
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

/* Measured IMU angular velocity (gx,gy,gz), acceleration (ax,ay,az) and magnetometer (mx,my,mz). */
struct FullIMUMeasurement {
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
	float mx;
	float my;
	float mz;
}


/******************************************************************************
LSM9DS1_Registers.h
SFE_LSM9DS1 Library - LSM9DS1 Register Map
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 21, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file defines all registers internal to the gyro/accel and magnetometer
devices in the LSM9DS1.

Development environment specifics:
	IDE: Arduino 1.6.0
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

Distributed as-is; no warranty is given.
******************************************************************************/

/////////////////////////////////////////
// LSM9DS1 Sizes Registers //
/////////////////////////////////////////
#define GYR_ACC_SIZE 6
#define MAG_SIZE 6
/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define ACT_THS				0x04
#define ACT_DUR				0x05
#define INT_GEN_CFG_XL		0x06
#define INT_GEN_THS_X_XL	0x07
#define INT_GEN_THS_Y_XL	0x08
#define INT_GEN_THS_Z_XL	0x09
#define INT_GEN_DUR_XL		0x0A
#define REFERENCE_G			0x0B
#define INT1_CTRL			0x0C
#define INT2_CTRL			0x0D
#define WHO_AM_I_XG			0x0F
#define CTRL_REG1_G			0x10
#define CTRL_REG2_G			0x11
#define CTRL_REG3_G			0x12
#define ORIENT_CFG_G		0x13
#define INT_GEN_SRC_G		0x14
#define OUT_TEMP_L			0x15
#define OUT_TEMP_H			0x16
#define STATUS_REG_0		0x17
#define OUT_X_L_G			0x18
#define OUT_X_H_G			0x19
#define OUT_Y_L_G			0x1A
#define OUT_Y_H_G			0x1B
#define OUT_Z_L_G			0x1C
#define OUT_Z_H_G			0x1D
#define CTRL_REG4			0x1E
#define CTRL_REG5_XL		0x1F
#define CTRL_REG6_XL		0x20
#define CTRL_REG7_XL		0x21
#define CTRL_REG8			0x22
#define CTRL_REG9			0x23
#define CTRL_REG10			0x24
#define INT_GEN_SRC_XL		0x26
#define STATUS_REG_1		0x27
#define OUT_X_L_XL			0x28
#define OUT_X_H_XL			0x29
#define OUT_Y_L_XL			0x2A
#define OUT_Y_H_XL			0x2B
#define OUT_Z_L_XL			0x2C
#define OUT_Z_H_XL			0x2D
#define FIFO_CTRL			0x2E
#define FIFO_SRC			0x2F
#define INT_GEN_CFG_G		0x30
#define INT_GEN_THS_XH_G	0x31
#define INT_GEN_THS_XL_G	0x32
#define INT_GEN_THS_YH_G	0x33
#define INT_GEN_THS_YL_G	0x34
#define INT_GEN_THS_ZH_G	0x35
#define INT_GEN_THS_ZL_G	0x36
#define INT_GEN_DUR_G		0x37

///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define OFFSET_X_REG_L_M	0x05
#define OFFSET_X_REG_H_M	0x06
#define OFFSET_Y_REG_L_M	0x07
#define OFFSET_Y_REG_H_M	0x08
#define OFFSET_Z_REG_L_M	0x09
#define OFFSET_Z_REG_H_M	0x0A
#define WHO_AM_I_M			0x0F
#define CTRL_REG1_M			0x20
#define CTRL_REG2_M			0x21
#define CTRL_REG3_M			0x22
#define CTRL_REG4_M			0x23
#define CTRL_REG5_M			0x24
#define STATUS_REG_M		0x27
#define OUT_X_L_M			0x28
#define OUT_X_H_M			0x29
#define OUT_Y_L_M			0x2A
#define OUT_Y_H_M			0x2B
#define OUT_Z_L_M			0x2C
#define OUT_Z_H_M			0x2D
#define INT_CFG_M			0x30
#define INT_SRC_M			0x30
#define INT_THS_L_M			0x32
#define INT_THS_H_M			0x33

////////////////////////////////
// LSM9DS1 WHO_AM_I Responses //
////////////////////////////////
#define WHO_AM_I_AG_RSP		0x68
#define WHO_AM_I_M_RSP		0x3D


// Prototype definitions
// ====================================================================================================================

// Initialization
/**
 *  Initialization of IMU:
 * 1 - Check WHO AM I if TRUE IMU attached and communication working
 * 2 - Initialize all registers to expected values
 * 3 - Set BIAS values at 0
 */
int initIMU();

// data reading
/**
 * Start of IMU:
 * 1 - Check if new data available
 * 2 - if so read corresponding registers
 * 3 - if Autocalc enabled compute bias TRUEed values
 *
 * TIME = �3772 us or 2452000 clock cycles
 */

/* Read data from IMU, remove bias and return as an IMUMeasurement. */
IMUMeasurement readIMU();

/**
 * calculates gx, gy, gz [rad/s] / (ADC tick)
 */
void readGyro();

/**
 * calculates ax, ay, az [g's] / (ADC tick)
 */
void readAcc();

/**
 * calculates magnetic field mag_x, mag_y, mag_z
 */
void readMag();

// void readTemp();

// AHRS calculation
/**
 * Calculate the attitude based on the provided accelerometer and magnetic data
 */
void calcAttitude(float ax, float ay, float az, float mx, float my, float mz);

// Internally used methods
//void calibrate(char autoCalc);

/**
 * Execute a step in the calibration of the IMU. Returns whether calibration is complete.
 */
bool calibrateIMU();

/**
 * Calibrate the Magnetometer
 */
void calibrateMag(char loadIn);

/**
 * Store the magnetic offset in the IMU registers
 */
void magOffset(uint8_t axis, int16_t offset);
//void enableFIFO(char enable);
//void setFIFO(int fifoMode, uint8_t fifoThs);

// This function reads in a signed 16-bit value and returns the scaled DPS/g's/Gauss
/**
 * Calculate the correct rad/ps for the provided IMU measurement
 */
float calcGyro(int gyro);

/**
 * Calculate the correct g's for the provided IMU measurement
 */
float calcAccel(int accel);

/**
 * Calculate the correct tesla's for the provided IMU measurement
 */
float calcMag(int mag);

// data scaling
//void calcgRes();
//void calcaRes();

/**
 * Get the magnetic sensitivity
 */
void calcmRes();

// data availability
/**
 * check availability of acceleration, gyroscope, temperature and magnetic data
 */
int AccDataAvailable();
int GyrDataAvailable();
int TempDataAvailable();
int MagDataAvailable();

// Data Buffers
u8 gyr_data[GYR_ACC_SIZE];
u8 acc_data[GYR_ACC_SIZE];
u8 mag_data[MAG_SIZE];
u8 temp[1];

// RAW 16 bit signed data from readings
int16_t acc_x, acc_y, acc_z;
int16_t gyr_x, gyr_y, gyr_z;
int16_t mag_x, mag_y, mag_z;
float g_smooth_x, g_smooth_y, g_smooth_z;

//Convert from RAW signed 16-bit value to gravity (g's)./Rad per second/Gauss (Gs)
/*float ax, ay, az, gx, gy, gz,*/
float mx, my, mz;

// gRes, aRes, and mRes store the current resolution for each sensor.
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15).
float gRes, aRes, mRes;
int scale_a, scale_g, scale_m;
int16_t temperature; // Chip temperature

// Measurement bias
float gBias[3], aBias[3], mBias[3];
float gBiasRaw[3], aBiasRaw[3], mBiasRaw[3], accBiasRaw[3];
long aBiasRawTemp[3],  gBiasRawTemp[3];

// _autoCalc keeps track of whether we're automatically subtracting off
// accelerometer and gyroscope bias calculated in calibrate().
char _autoCalc, _magCalc;

// Initialization counters
int ii, ahrs;

#endif /* IMU_H_ */
