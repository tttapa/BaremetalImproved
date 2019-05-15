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
    TEST_MANUAL = 0,

    /**
     * Manual mode and altitude-hold mode are enabled. However, switching to
     * autonomous mode has no effect.
     */
    TEST_ALTITUDE_HOLD = 1,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the air and the drone will be stuck in the LOITERING state until the
     * pilot switches back to altitude-hold mode.
     */
    TEST_LOITERING = 2,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the air. The drone will loiter for 15 seconds, then it remain in
     * NAVIGATING and CONVERGING until the pilot switches back to altitude-hold
     * mode. During this time, the drone will cycle through an array of
     * prespecified targets, navigating and converging on each one.
     */
    TEST_NAVIGATION = 3,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the air. The drone will loiter for 15 seconds, then it will perform
     * the landing routine. It will remain in the IDLE_GROUND state until the
     * pilot switches back to altitude-hold mode.
     */
    TEST_LANDING = 4,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the air. The drone will loiter for 15 seconds, then it will read the
     * first QR code as soon as the drone is stable. After decrypting the QR
     * code, it will fly to the second QR code, but will arrive 1 tile off. When
     * ANC asks to scan the current QR code, IMP will respone that there is no
     * QR code below the drone. Then the drone will perform a spiral search to
     * find the QR code, and then it will land.
     */
    TEST_QR_WALKING = 5,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the air. The drone will loiter for 15 seconds, then it will read the
     * first QR code as soon as the drone is stable. After decrypting the QR
     * code, it will fly to the second QR code, but will arrive 1 tile off. When
     * ANC asks to scan the current QR code, IMP will respone that there is no
     * QR code below the drone. Then the drone will perform a spiral search to
     * find the QR code, and then it will land.
     */
    TEST_QR_NAVIGATION_LOST = 6,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the ground. The drone will perform a pre-takeoff routine and
     * maintain the signals that were sent in the last iteration of the pre-
     * takeoff routine until the pilot switches back to altitude-hold mode.
     */
    TEST_PRETAKEOFF = 7,

    /**
     * All flight modes are active, but autonomous mode can only be activated
     * from the ground. The drone will perform a pre-takeoff routine, then it
     * will perform the takeoff routine, and finally it will remain in LOITERING
     * until the pilot switches back to altitude-hold mode.
     */
    TEST_TAKEOFF = 8,

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
    DEMO = 9,

};

/** Get the drone's test mode. */
TestMode getTestMode();

/** Get whether switching to altitude mode is enabled. */
bool canStartAltitudeHoldMode();

/** Get whether switching to autonomous mode is enabled. */
bool canStartAutonomousMode();

/** Get whether switching to autonomous mode from the air is enabled. */
bool canStartAutonomousModeAir();

/** Get whether switching to autonomous mode from the ground is enabled. */
bool canStartAutonomousModeGround();

/** Get the next navigation target during TEST_NAVIGATION mode. */
Position getNextNavigationTestTarget();

/** Get whether switching to altitude mode is enabled. */
bool isAltitudeHoldModeEnabled();

/** Get whether autonomous mode can be activated from the air. */
bool isAutonomousAirEnabled();

/** Get whether autonomous mode can be activated from the ground. */
bool isAutonomousGroundEnabled();

/** Get whether the drone can switch from LOITERING to CONVERGING/NAVIGATING. */
bool isNavigationEnabled();

/**
 * Get whether the drone can switch from LOITERING to CONVERGING/NAVIGATING,
 * using QR codes to navigate.
 */
bool isNavigationEnabledQRCodes();

/**
 * Get whether the drone can switch from LOITERING to CONVERGING/NAVIGATING,
 * using prespecified test targets to navigate.
 */
bool isNavigationEnabledTestTargets();

/** Get whether the drone is able to land. */
bool isLandingEnabled();

/** Get whether the drone should loiter indefinitely after taking off. */
bool shouldLoiterIndefinitelyAfterTakeoff();

/** Get whether the drone should loiter indefinitely with air initialization. */
bool shouldLoiterIndefinitelyWithInitAir();

/**
 * Get whether the loitering timer should be shortened. This is only the case
 * during TEST_QR_WALKING mode.
 */
bool shouldLoiteringTimerBeShortened();

/** Get the whether the drone should take off after pretakeoff. */
bool shouldTakeOffAfterPreTakeoff();

/**
 * Get whether the spiral search will be tested after reading the first QR code.
 */
bool shouldTestQRSearch();
