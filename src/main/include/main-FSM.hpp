#pragma once

/**
 * Update the main FSM for flying. Should be called on every interrupt 
 * (@238 Hz) 
 */
void updateMainFSM();

/**
 * An enumeration for the different flight modes.
 */
enum class FlightMode {
    /** The RC controls the reference orientation and thrust directly */
    Manual = 0,
    /** The RC controls the reference orientation directly, but the thrust is
     * controlled by the altitude controller. Using the throtle control on the
     * RC, the reference altitude can be set. */
    AltitudeHold = 1,
    /** The reference orientation is controlled by the position controller, and 
     * the thrust is controlled by the altitude controller.
     * The altitude and position setpoints are controlled by the autonomous mode 
     * FSM. */
    Autonomous = 2,
    /** Let's all hope this never happens. */
    Error = 3,
};

/** Convert a flight mode enumerator to its string representation. */
inline const char *toString(FlightMode mode) {
    switch (mode) {
        case FlightMode::Manual: return "Manual";
        case FlightMode::AltitudeHold: return "AltitudeHold";
        case FlightMode::Autonomous: return "Autonomous";
        case FlightMode::Error: return "Error";
        default: return "(invalid)";
    }
}

/** Get the flight mode we're currently in. */
FlightMode getFlightMode();