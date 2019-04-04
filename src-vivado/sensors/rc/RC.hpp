// Original: BareMetal/src/RC/RC.h
// Original: BareMetal/src/control/eagle1globals.h
/**********************************************************************************************************************
*   Radio Control header file
*   This file contains all parameters used to read input from the RC.
*   This file should normally not be changed by the students.
*   author: w. devries, p. coppens
***********************************************************************************************************************/


// Constant definitions
// ====================================================================================================================
// The following constants map to the XPAR parameters created in the
// xparameters.h file. They are only defined here such that a user can easily
// change all the needed parameters in one place


// Prototype definitions
// ====================================================================================================================


namespace INDUCTIVE {
    const int ON = 1;
    const int OFF = 2;
}


namespace MODE {
    const int MANUAL = 1;
    const int ALTITUDE_HOLD = 2;
    const int AUTONOMOUS = 3;
}


struct RCInput {
    float throttle;
    float roll;
    float pitch;
    float yaw;
    float tuner;
    int mode;
    int inductive;
};

/**
 * Read the RC voltages from the registers and assign them to the correct instances
 */
RCInput readRC();

