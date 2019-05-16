#pragma once

/* Includes from src. */
#include <BaremetalCommunicationDef.hpp>  ///< FlightMode, WPTMode
#include <SensorTypes.hpp>                ///< RCInput
#include <real_t.h>

/**
 * Get the current RC flight mode.
 */
FlightMode getFlightMode();

/**
 * Get the current RC pitch in [-1, 1].
 */
real_t getPitch();

/**
 * Get the current RC roll in [-1, 1].
 */
real_t getRoll();

/**
 * Get the current RC throttle in [0, 1].
 */
real_t getThrottle();

/**
 * Get the current RC tuner knob value in [-1, 1].
 */
real_t getTuner();

/**
 * Get the current RC WPT mode.
 */
WPTMode getWPTMode();

/**
 * Get the current RC yaw in [-1, 1].
 */
real_t getYaw();

/**
 * Set the RC manager's RCInput to the given RCInput.
 * 
 * @param   this->rcInput
 *          New RCInput, see RC.hpp.
 */
void setRCInput(RCInput rcInput);
