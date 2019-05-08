#pragma once
#include <BaremetalCommunicationDef.hpp>
#include <SensorTypes.hpp>
#include <real_t.h>

/**
 * Get the current RC flight mode.
 */
FlightMode getFlightMode();

/**
 * Get the current RC pitch.
 */
real_t getPitch();

/**
 * Get the current RC roll.
 */
real_t getRoll();

/**
 * Get the current RC throttle.
 */
real_t getThrottle();

/**
 * Get the current RC tuner knob value.
 */
real_t getTuner();

/**
 * Get the current RC WPT mode.
 */
WPTMode getWPTMode();

/**
 * Get the current RC yaw.
 */
real_t getYaw();

/**
 * Set the RC manager's RCInput to the given RCInput.
 * 
 * @param   this->rcInput
 *          New RCInput, see RC.hpp.
 */
void setRCInput(RCInput rcInput);
