#pragma once

/* Includes from src. */
#include <BaremetalCommunicationDef.hpp>  ///< Position
#include <Quaternion.hpp>
#include <SensorTypes.hpp>
#include <real_t.h>

// TODO: comment this...
real_t getYawJump();

AccelMeasurement getAccelMeasurement();

GyroMeasurement getGyroMeasurement();

IMUMeasurement getIMUMeasurement();

Quaternion getAHRSQuat();

Quaternion getJumpedAHRSQuat();

real_t getSonarMeasurement();

real_t getCorrectedSonarMeasurement();

real_t getYawMeasurement();

Position getPositionMeasurement();

Position getCorrectedPositionMeasurement();

void setYawJump(real_t value);

void setIMUMeasurement(IMUMeasurement value);

void setAHRSQuat(Quaternion value);

void setJumpedAHRSQuat(Quaternion value);

void setSonarMeasurement(real_t value);

void setCorrectedSonarMeasurement(real_t value);

void setYawMeasurement(real_t value);

void setPositionMeasurement(Position value);

void setCorrectedPositionMeasurement(Position value);
