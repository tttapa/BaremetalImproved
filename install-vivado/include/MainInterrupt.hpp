#pragma once
#include <LoggerStructs.hpp>

/** Whether an interrupt is currently running. */
extern volatile bool isInterruptRunning;

/**
 * @file    MainInterrupt.hpp
 * @brief   This file contains an update function that will be called whenever 
 *          the IMU interrupts Main.cpp.
 */

// TODO: comments
float calculateYawJump(float yaw);

AttitudeReference calculatePositionControllerOutput(float yawMeasurement, float yawRef, float pitchRef, float rollRef);

void mainOperation();

void updateFSM();