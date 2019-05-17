#pragma once
#include <cstdint>

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
float getTime();

/**
 * Get the number of milliseconds since boot.
 */
uint64_t getMillis();
