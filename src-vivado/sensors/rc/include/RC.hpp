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
 * Struct containing the four different flight modes. First, the drone begins in
 * the UNINITIALIZED flight mode. After the first cycle, the drone will enter
 * the current flight mode, as specified by the RC. In MANUAL_MODE, the pilot
 * has full control over the drone's orientation and the common thrust. In
 * ALTITUDE_HOLD_MODE, the pilot still has control over the drone's orientation,
 * but the altitude controller takes over the common thrust in order to keep the
 * drone at a constant height. Finally, in AUTONOMOUS_MODE, the pilot has no
 * control over the attitude or altitude of the drone. The drone will navigate
 * autonomously to successive QR codes and land at its final code. As a safety
 * precaution, if the pilot sets the throttle to zero during the autonomous
 * flight, the drone will land at its current location.
 */
enum FlightMode {

    /**
     * The drone is in its first clock cycle and has not yet entered a flight
     * mode.
     */
    UNINITIALIZED = 0,

    /**
     * The drone is in "manual mode". The pilot has control over the drone's
     * orientation and the common thrust.
     */
    MANUAL_MODE = 1,

    /**
     * The drone is in "altitude-hold mode". The pilot has control over drone's
     * orientation, but the altitude controller takes over the common thrust in
     * order to keep the drone at a constant altitude.
     */
    ALTITUDE_HOLD_MODE = 2,

    /**
     * The drone is in "autonomous mode". The pilot has no control over the
     * attitude or altitude of the drone. If the drone is grounded when entering
     * this flight mode, then it will take off as soon as the pilot raises the
     * throttle above the predetermined threshold (see Autonomous.hpp). If the
     * drone was already airborne when entering this flight mode, then this step
     * will be skipped. Then, the drone will loiter at its current position for
     * a predetermined time (see Autonomous.hpp). After that, it will navigate
     * autonomously to successive QR codes and finally land at its final code.
     * As a safety precaution, if the pilot sets the throttle to zero during the
     * autonomous flight, the drone will land at its current location.
     */
    AUTONOMOUS_MODE = 3,
};

/**
 * Read the RC voltages from the registers.
 * 
 * @return  A struct containing values for the RC throttle, roll, pitch, yaw,
 *          each of which is in [0,1], for the tuner knob, which is in
 *          [-0.5,+0,5], and for the flight mode and WPT mode (enumerations).
 */
RCInput readRC();
