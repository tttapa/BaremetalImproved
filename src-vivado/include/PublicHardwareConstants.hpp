#pragma once
/**
 * Seconds per tick of the IMU in seconds. This value is set when the IMU is
 * initialized in src-vivado/sensors/IMU.cpp.
 */
extern float SECONDS_PER_TICK;

/**
 * Frequency of the IMU in Hz. This value is set when the IMU is initialized in
 * src-vivado/sensors/IMU.cpp.
 */
extern float TICKS_PER_SECOND;

/** The multiple of 119 Hz at which the IMU is set. */
extern float IMU_FACTOR;

/** Frequency of the sonar measurements is 20.0 Hz. */
const float SONAR_FREQUENCY = 20.0;