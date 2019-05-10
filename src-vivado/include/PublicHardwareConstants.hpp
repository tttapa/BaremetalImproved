#pragma once
/**
 * Seconds per tick of the IMU in seconds. This value is set when the IMU is
 * initialized in src-vivado/sensors/IMU.cpp.
 */
float SECONDS_PER_TICK;

/**
 * Frequency of the IMU in Hz. This value is set when the IMU is initialized in
 * src-vivado/sensors/IMU.cpp.
 */
float TICKS_PER_SECOND;

/** Frequency of the sonar measurements is 20.0 Hz. */
const float SONAR_FREQUENCY = 20.0;