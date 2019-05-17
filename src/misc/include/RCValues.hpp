#pragma once

/* Includes from src. */
#include <LoggerStructs.hpp>              ///< RCInput

/** Get the current RC flight mode. */
FlightMode getFlightMode();

/** Get the current RC pitch in [-1, 1]. */
float getPitch();

/** Return the entire RC input. */
RCInput getRCInput();

/** Get the current RC roll in [-1, 1]. */
float getRoll();

/** Get the current RC throttle in [0, 1]. */
float getThrottle();

/** Get the current RC tuner knob value in [-1, 1]. */
float getTuner();

/** Get the current RC WPT mode. */
WPTMode getWPTMode();

/** Get the current RC yaw in [-1, 1]. */
float getYaw();

/**
 *  Set the RC manager's RCInput to the given RCInput.
 * 
 * @param   this->rcInput
 *          New RCInput, see RC.hpp.
 */
void setRCInput(RCInput rcInput);
