#pragma once

/** Whether an interrupt is currently running. */
extern volatile bool isInterruptRunning;

/**
 * @file    MainInterrupt.hpp
 * @brief   This file contains an update function that will be called whenever 
 *          the IMU interrupts `Main.cpp`.
 */

void updateFSM();
