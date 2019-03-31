#pragma once

/**********************************************************************************************************************
*   This file maintains the drone's internal clock.
*   Author:
***********************************************************************************************************************/

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
float getTime();