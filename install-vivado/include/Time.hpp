#pragma once
#include <cstdint>

/* Includes from src. */
#include <real_t.h>

/** Clock is updated when interrupts occur at 238 Hz. */
const real_t TICKS_PER_SECOND = 238.0;
const real_t SECONDS_PER_TICK = 1.0 / TICKS_PER_SECOND;

/**
 * Increment the tick counter.
 */
void incrementTickCount();

/**
 * Get the tick counter;
 */
uint32_t getTickCount();

/**
 * Get the time since startup in seconds.
 */
real_t getTime();

/**
 * Get the number of milliseconds since boot.
 */
uint64_t getMillis();
