#pragma once
#include <BaremetalCommunicationDef.hpp>
#include <real_t.h>

/**
 * Struct containing the values from the RC transmitter. This includes the
 * value of the throttle, roll, pitch and yaw, which range from 0 to 1. It
 * also contains the value of the tuner knob, which ranges from -0.5 to +0.5.
 * Lastly there are switches for the flight mode and the wireless power
 * transfer. These are represented by their respective enumerations.
 */
struct RCInput {
    real_t throttle;        ///< Value of the RC throttle in [0,1].
    real_t roll;            ///< Value of the RC roll in [0,1].
    real_t pitch;           ///< Value of the RC pitch in [0,1].
    real_t yaw;             ///< Value of the RC yaw in [0,1].
    real_t tuner;           ///< Value of the RC tuner knob in [-0.5,+0.5].
    FlightMode flightMode;  ///< Value of the RC flight mode (as a FlightMode).
    WPTMode wptMode;        ///< Value of the RC WPT mode (as a WPTMode).
};

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
