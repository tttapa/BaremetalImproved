// Original: BareMetal/src/RC/RC.h
// Original: BareMetal/src/control/eagle1globals.h
/**********************************************************************************************************************
*   Radio Control header file
*   This file contains all parameters used to read input from the RC.
*   This file should normally not be changed by the students.
*   author: w. devries, p. coppens
***********************************************************************************************************************/
#pragma once

// Constant definitions
// ====================================================================================================================
// The following constants map to the XPAR parameters created in the
// xparameters.h file. They are only defined here such that a user can easily
// change all the needed parameters in one place

// Prototype definitions
// ====================================================================================================================

struct RCInput {
    float throttle;
    float roll;
    float pitch;
    float yaw;
    float tuner;
    int mode;
    int inductive;
};

enum InductiveMode {
    ON  = 1,
    OFF = 2,
};

enum FlightMode {
    MANUAL_MODE        = 1,
    ALTITUDE_HOLD_MODE = 2,
    AUTONOMOUS_MODE    = 3,
};

/**
 * Read the RC voltages from the registers and assign them to the correct instances
 */
RCInput readRC();
