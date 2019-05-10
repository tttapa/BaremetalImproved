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

/** Get the corrected position measurement for logger. */
Position getCorrectedPositionMeasurement();

/** Get the gyroscope measurement for logger. */
GyroMeasurement getGyroMeasurement();

/** Get the IMU measurement for logger. */
IMUMeasurement getIMUMeasurement();

/** Get the jumped AHRS quaternion for logger. */
Quaternion getJumpedAHRSQuat();

/** Get the motor signals for the logger. */
MotorSignals getMotorSignals();

/** Get the sonar measurement for logger. */
real_t getSonarMeasurement();

/** Get the corrected sonar measurement for logger. */
real_t getCorrectedSonarMeasurement();

/** Get the yaw jump for logger. */
real_t getYawJump();

/** Get the yaw measurement for logger. */
real_t getYawMeasurement();

/** Get the position measurement for logger. */
Position getPositionMeasurement();

/** Set the AHRS quaternion for logger. */
void setAHRSQuat(Quaternion value);

/** Set the common thrust for the logger. */
void setCommonThrust(real_t value);

/** Set the corrected position measurement for logger. */
void setCorrectedPositionMeasurement(Position value);

/** Set the corrected sonar measurement for logger. */
void setCorrectedSonarMeasurement(real_t value);

/** Set the IMU measurement for logger. */
void setIMUMeasurement(IMUMeasurement value);

/** Set the jumped AHRS quaternion for logger. */
void setJumpedAHRSQuat(Quaternion value);

/** Set the motor signals for the logger. */
void setMotorSignals(MotorSignals value);

/** Set the position measurement for logger. */
void setPositionMeasurement(Position value);

/** Set the sonar measurement for logger. */
void setSonarMeasurement(real_t value);

/** Set the yaw jump for logger. */
void setYawJump(real_t value);

/** Set the yaw measurement for logger. */
void setYawMeasurement(real_t value);
