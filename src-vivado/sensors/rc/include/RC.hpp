// Original: BareMetal/src/RC/RC.h
// Original: BareMetal/src/control/eagle1globals.h
/**********************************************************************************************************************
*   Radio Control header file
*   This file contains all parameters used to read input from the RC.
*   This file should normally not be changed by the students.
*   author: w. devries, p. coppens
***********************************************************************************************************************/
#pragma once
#include "../../../../src/communication/include/BaremetalCommunicationDef.hpp"

// Constant definitions
// ====================================================================================================================
// The following constants map to the XPAR parameters created in the
// xparameters.h file. They are only defined here such that a user can easily
// change all the needed parameters in one place

// Prototype definitions
// ====================================================================================================================

/**
 * Struct containing the values from the RC transmitter. This includes the
 * value of the throttle, roll, pitch and yaw, which range from 0 to 1. It
 * also contains the value of the tuner knob, which ranges from -0.5 to +0.5.
 * Lastly there are switches for the flight mode and the wireless power
 * transfer. These are represented by their respective enumerations.
 */
struct RCInput {
    float throttle;         ///< Value of the RC throttle in [0,1].
    float roll;             ///< Value of the RC roll in [0,1].
    float pitch;            ///< Value of the RC pitch in [0,1].
    float yaw;              ///< Value of the RC yaw in [0,1].
    float tuner;            ///< Value of the RC tuner knob in [-0.5,+0.5].
    FlightMode flightMode;  ///< Value of the RC flight mode (as a FlightMode).
    WPTMode wptMode;        ///< Value of the RC WPT mode (as a WPTMode).
};

/**
 * Struct containing the two different modes or statuses of the Wireless Power
 * Transfer: OFF and ON.
 */
enum WPTMode {
    OFF = 0,  ///< Wireless Power Transfer is turned off.
    ON  = 1,  ///< Wireless Power Transfer is turned on.
};

/**
 * Read the RC voltages from the registers.
 * 
 * @return  A struct containing values for the RC throttle, roll, pitch, yaw,
 *          each of which is in [0,1], for the tuner knob, which is in
 *          [-0.5,+0,5], and for the flight mode and WPT mode (enumerations).
 */
RCInput readRC();
