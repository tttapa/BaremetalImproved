#pragma once
#include "../../../src-vivado/sensors/imu/include/IMU.hpp"
#include <Quaternion.hpp>
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

real_t getPositionMeasurement();

real_t getCorrectedPositionMeasurement();

void setYawJump(real_t value);

void setIMUMeasurement(IMUMeasurement value);

void setAHRSQuat(Quaternion value);

void setJumpedAHRSQuat(Quaternion value);

void setSonarMeasurement(real_t value);

void setCorrectedSonarMeasurement(real_t value);

void setYawMeasurement(real_t value);

void setPositionMeasurement(real_t value);

void setCorrectedPositionMeasurement(real_t value);
