// Original: BareMetal/src/RC/RC.h
// Original: BareMetal/src/control/eagle1globals.h
/**********************************************************************************************************************
*   Radio Control header file
*   This file contains all parameters used to read input from the RC.
*   This file should normally not be changed by the students.
*   author: w. devries, p. coppens
***********************************************************************************************************************/
#pragma once
#include "../../../../src/globals/include/Globals.hpp"

// Constant definitions
// ====================================================================================================================
// The following constants map to the XPAR parameters created in the
// xparameters.h file. They are only defined here such that a user can easily
// change all the needed parameters in one place



// Prototype definitions
// ====================================================================================================================

/**
 * Read the RC voltages from the registers.
 * 
 * @return  A struct containing values for the RC throttle, roll, pitch, yaw,
 *          each of which is in [0,1], for the tuner knob, which is in
 *          [-0.5,+0,5], and for the flight mode and WPT mode (enumerations).
 */
RCInput readRC();
