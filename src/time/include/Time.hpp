#pragma once
#include <real_t.h>

/**********************************************************************************************************************
*   This file maintains the drone's internal clock.
*   Author:
***********************************************************************************************************************/

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
int getTickCount();

/**
 * Get the time since startup in seconds.
 */
real_t getTime();