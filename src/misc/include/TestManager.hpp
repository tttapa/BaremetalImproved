#pragma once

/**
 * Various modes with which to test the drone's functionality. These include
 * MANUAL, ALTITUDE_HOLD, LOITERING, NAVIGATING, LANDING, PRETAKEOFF, TAKEOFF,
 * and DEMO. It is advised to test it in this order, skipping PRETAKEOFF and
 * TAKEOFF if you're short of time.
 */
enum TestMode {

    /**
     * Only manual mode is enabled on the drone. Mode switching has no effect.
     */
    MANUAL = 0,

    /**
     * Manual mode and altitude-hold mode are enabled. However, switching to
     * autonomous mode has no effect.
     */
    ALTITUDE_HOLD = 1,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the air and the drone will be stuck in the LOITERING state until the
     * pilot switches back to altitude-hold mode.
     */
    LOITERING = 2,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the air. The drone will loiter for 15 seconds, then it remain in
     * NAVIGATING and CONVERGING until the pilot switches back to altitude-hold
     * mode. During this time, the drone will cycle through an array of
     * prespecified targets, navigating and converging on each one.
     */
    NAVIGATING = 3,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the air. The drone will loiter for 15 seconds, then it will perform
     * the landing routine. It will remain in the IDLE_GROUND state until the
     * pilot switches back to altitude-hold mode.
     */
    LANDING = 4,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the ground. The drone will perform a pre-takeoff routine and
     * maintain the signals that were sent in the last iteration of the pre-
     * takeoff routine until the pilot switches back to altitude-hold mode.
     */
    PRETAKEOFF = 5,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the ground. The drone will perform a pre-takeoff routine, then it
     * will perform the takeoff routine, and finally it will remain in LOITERING
     * until the pilot switches back to altitude-hold mode.
     */
    TAKEOFF = 6,

    /**
     * All flight modes are active, and autonomous mode can be activated from
     * the ground or from the air. [If autonomous mode is activated from the
     * ground, the drone will first preform the pre-takeoff and the takeoff
     * routines.] Then, the drone will loiter for 15 seconds. After that it will
     * attempt to follow the QR trail until it receives the QR_LAND instruction,
     * where it will land and remain in IDLE_GROUND until WPT is activated, the
     * drone is told to take off again, or the pilot switches back to altitude-
     * hold mode.
     */
    DEMO = 7,

};

/** Get the drone's test mode. */
TestMode getTestMode();

/** Get whether switching to altitude mode is enabled. */
bool isAltitudeHoldModeEnabled();

/** Get whether autonomous mode can be activated from the air. */
bool isAutonomousAirEnabled();

/** Get whether autonomous mode can be activated from the ground. */
bool isAutonomousGroundEnabled();

/** Get whether switching to autonomous mode is enabled. */
bool isAutonomousModeEnabled();

/** Get whether the drone can switch from LOITERING to CONVERGING/NAVIGATING. */
bool isNavigatingEnabled();

/** Get whether the drone should land after receiving a QR_LAND flag. */
bool isQRLandingEnabled();

/** Get whether the drone should switch from LOITERING to LANDING. */
bool shouldLandAfterLoitering();
