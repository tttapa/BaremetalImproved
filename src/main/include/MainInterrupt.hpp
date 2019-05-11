#pragma once
#include <real_t.h>

/** Whether an interrupt is currently running. */
extern volatile bool isInterruptRunning;


/**
 * @file    MainInterrupt.hpp
 * @brief   This file contains an update function that will be called whenever 
 *          the IMU interrupts `Main.cpp`.
 */

// TODO: comments
real_t calculateYawJump(float yaw);

void mainOperation();

void updateFSM();