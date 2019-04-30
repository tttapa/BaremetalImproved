#pragma once
#include <real_t.h>

/**********************************************************************************************************************
*   This file maintains the drone's internal clock.
*   Author:
***********************************************************************************************************************/

/**
 * Increment the tick counter.
 */
void incrementTickCount();

/**
 * Returns the time between ticks in seconds, namely (1.0/238.0) seconds.
 */
real_t getSecondsPerTick();

/**
 * Get the tick counter;
 */
int getTickCount();

/**
 * Get the time since startup in seconds.
 */
real_t getTime();