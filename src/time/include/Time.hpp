#pragma once
#include <real_t.h>
#include <cstdint>

/** Clock is updated when interrupts occur at 238 Hz. */
// TODO: const or constexpr for file constants?
static const real_t SECONDS_PER_TICK = 1.0 / 238.0;

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