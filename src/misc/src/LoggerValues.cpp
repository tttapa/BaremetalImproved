#include <LoggerValues.hpp>

#pragma region Variables
/** Current value of the AHRS quaternion. */
static Quaternion ahrsQuat;

/** Current value of the common thrust. */
real_t commonThrust;

/** Current value of the corrected position measurement. */
static Position correctedPositionMeasurement;

/** Current value of the corrected sonar measurement. */
static real_t correctedSonarMeasurement;

/** Current value of the IMU measurement (gyro + accel). */
static IMUMeasurement imuMeasurement;

/** Current value of the jumped AHRS quaternion. */
static Quaternion jumpedAhrsQuat;

/** Current duty cycles sent to the 4 ESCs. */
MotorSignals motorSignals;

/** Current value of the position measurement. */
static Position positionMeasurement;

/** Current value of the sonar measurement. */
static real_t sonarMeasurement;

/** Current value of the yaw measurement. */
static real_t yawMeasurement;

/** Current value of the yaw jump in radians. */
static real_t yawJump;
#pragma endregion

#pragma region Getters
AccelMeasurement getAccelMeasurement() {
    return {imuMeasurement.ax, imuMeasurement.ay, imuMeasurement.az};
}

Quaternion getAHRSQuat() { return ahrsQuat; }

real_t getCommonThrust() { return commonThrust; }

Position getCorrectedPositionMeasurement() {
    return correctedPositionMeasurement;
}

real_t getCorrectedSonarMeasurement() { return correctedSonarMeasurement; }

GyroMeasurement getGyroMeasurement() {
    return {imuMeasurement.gx, imuMeasurement.gy, imuMeasurement.gz};
}

IMUMeasurement getIMUMeasurement() { return imuMeasurement; }

Quaternion getJumpedAHRSQuat() { return jumpedAhrsQuat; }

MotorSignals getMotorSignals() { return motorSignals; }

real_t getSonarMeasurement() { return sonarMeasurement; }

real_t getYawJump() { return yawJump; }

real_t getYawMeasurement() { return yawMeasurement; }

Position getPositionMeasurement() { return positionMeasurement; }
#pragma endregion

#pragma region Setters
void setAHRSQuat(Quaternion value) { ahrsQuat = value; }

void setCommonThrust(real_t value) { commonThrust = value; }

void setCorrectedPositionMeasurement(Position value) {
    correctedPositionMeasurement = value;
}

void setCorrectedSonarMeasurement(real_t value) {
    correctedSonarMeasurement = value;
}

void setIMUMeasurement(IMUMeasurement value) { imuMeasurement = value; }

void setJumpedAHRSQuat(Quaternion value) { jumpedAhrsQuat = value; }

void setMotorSignals(MotorSignals value) { motorSignals = value; }

void setPositionMeasurement(Position value) { positionMeasurement = value; }

void setSonarMeasurement(real_t value) { sonarMeasurement = value; }

void setYawJump(real_t value) { yawJump = value; }

void setYawMeasurement(real_t value) { yawMeasurement = value; }
#pragma endregion
