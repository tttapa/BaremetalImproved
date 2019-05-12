#pragma once

/* Includes from src. */
#include <BaremetalCommunicationDef.hpp>  ///< Position
#include <ConfigurationManager.hpp>
#include <ControllerInstances.hpp>
#include <MiscInstances.hpp>
#include <OutputTypes.hpp>  ///< MotorSignals
#include <Quaternion.hpp>
#include <RCValues.hpp>  ///< RC flight mode, pitch, roll, tuner, WPT mode, yaw
#include <SensorTypes.hpp>  ///< AccelMeasurement, GyroMeasurement, IMUMeasurement
#include <Time.hpp>
#include <real_t.h>

/** Get the accelerometer measurement for logger. */
AccelMeasurement getAccelMeasurement();

/** Get the AHRS quaternion for logger. */
Quaternion getAHRSQuat();

/** Get the common thrust for the logger. */
real_t getCommonThrust();

/** Get the corrected position measurement for logger in meters. */
Position getCorrectedPositionMeasurement();

/** Get the corrected sonar measurement for logger in meters. */
real_t getCorrectedSonarMeasurement();

/** Get the gyroscope measurement for logger. */
GyroMeasurement getGyroMeasurement();

/** Get the IMU measurement for logger. */
IMUMeasurement getIMUMeasurement();

/** Get the jumped AHRS quaternion for logger. */
Quaternion getJumpedAHRSQuat();

/** Get the motor signals for the logger. */
MotorSignals getMotorSignals();

/** Get the position measurement for logger in meters. */
Position getPositionMeasurement();

/** Get the position measurement for logger in blocks. */
Position getPositionMeasurementBlocks();

/** Get the sonar measurement for logger in meters. */
real_t getSonarMeasurement();

/** Get the yaw jump for logger in radians. */
real_t getYawJump();

/** Get the yaw measurement for logger in radians. */
real_t getYawMeasurement();


/** Set the AHRS quaternion for logger. */
void setAHRSQuat(Quaternion value);

/** Set the common thrust for the logger. */
void setCommonThrust(real_t value);

/** Set the corrected position measurement for logger in meters. */
void setCorrectedPositionMeasurement(Position value);

/** Set the corrected sonar measurement for logger in meters. */
void setCorrectedSonarMeasurement(real_t value);

/** Set the IMU measurement for logger. */
void setIMUMeasurement(IMUMeasurement value);

/** Set the jumped AHRS quaternion for logger. */
void setJumpedAHRSQuat(Quaternion value);

/** Set the motor signals for the logger. */
void setMotorSignals(MotorSignals value);

/** Set the position measurement for logger in meters. */
void setPositionMeasurement(Position value);

/** Set the position measurement for logger in blocks. */
void setPositionMeasurementBlocks(Position value);

/** Set the sonar measurement for logger in meters. */
void setSonarMeasurement(real_t value);

/** Set the yaw jump for logger in radians. */
void setYawJump(real_t value);

/** Set the yaw measurement for logger in radians. */
void setYawMeasurement(real_t value);
