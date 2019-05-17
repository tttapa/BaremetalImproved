#include <PublicHardwareConstants.hpp>

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

/** The multiple of 119 Hz at which the IMU is set. */
float IMU_FACTOR;