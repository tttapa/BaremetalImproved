#include <SensorValues.hpp>

/** Current value of the yaw jump in radians. */
static real_t yawJump;

/** Current value of the IMU measurement (gyro + accel). */
static IMUMeasurement imuMeasurement;

/** Current value of the AHRS quaternion. */
static Quaternion ahrsQuat;

/** Current value of the jumped AHRS quaternion. */
static Quaternion jumpedAhrsQuat;

/** Current value of the sonar measurement. */
static real_t sonarMeasurement;

/** Current value of the corrected sonar measurement. */
static real_t correctedSonarMeasurement;

/** Current value of the yaw measurement. */
static real_t yawMeasurement;

/** Current value of the position measurement. */
static Position positionMeasurement;

/** Current value of the corrected position measurement. */
static Position correctedPositionMeasurement;

real_t getYawJump() { return yawJump; }

AccelMeasurement getAccelMeasurement() {
    return {imuMeasurement.ax, imuMeasurement.ay, imuMeasurement.az};
}

GyroMeasurement getGyroMeasurement() {
    return {imuMeasurement.gx, imuMeasurement.gy, imuMeasurement.gz};
}

IMUMeasurement getIMUMeasurement() { return imuMeasurement; }

Quaternion getAHRSQuat() { return ahrsQuat; }

Quaternion getJumpedAHRSQuat() { return jumpedAhrsQuat; }

real_t getSonarMeasurement() { return sonarMeasurement; }

real_t getCorrectedSonarMeasurement() { return correctedSonarMeasurement; }

real_t getYawMeasurement() { return yawMeasurement; }

Position getPositionMeasurement() { return positionMeasurement; }

Position getCorrectedPositionMeasurement() {
    return correctedPositionMeasurement;
}

void setYawJump(real_t value) { yawJump = value; }

void setIMUMeasurement(IMUMeasurement value) { imuMeasurement = value; }

void setAHRSQuat(Quaternion value) { ahrsQuat = value; }

void setJumpedAHRSQuat(Quaternion value) { jumpedAhrsQuat = value; }

void setSonarMeasurement(real_t value) { sonarMeasurement = value; }

void setCorrectedSonarMeasurement(real_t value) {
    correctedSonarMeasurement = value;
}

void setYawMeasurement(real_t value) { yawMeasurement = value; }

void setPositionMeasurement(Position value) { positionMeasurement = value; }

// TODO: calculate corrected position automatically
void setCorrectedPositionMeasurement(Position value) {
    correctedPositionMeasurement = value;
}
