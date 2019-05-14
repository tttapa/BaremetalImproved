#include <MiscInstances.hpp>
#include <TestMode.hpp>

/** Current test mode is MANUAL. */
static const TestMode TEST_MODE = TestMode::TEST_LOITERING;

static constexpr int NUM_NAVIGATION_TARGETS = 7;

/** Points to cycle through when during the NAVIGATION test mode. */
static const Position NAVIGATION_TARGETS[NUM_NAVIGATION_TARGETS] = {
    Position{0.0, 0.0}, //
Position{1.0, 0.0}, //
Position{1.0, 1.0}, //
Position{-1.0, 1.0}, //
Position{-1.0, -1.0}, //
Position{1.0, -1.0}, //
Position{1.0, 0.0}, //
};

/** Current navigation target index. */
static int navigationTargetIndex = 0;

/** Get the drone's test mode. */
TestMode getTestMode() { return TEST_MODE; }

/** Get whether switching to altitude mode is enabled. */
bool canStartAltitudeHoldMode() { return isAltitudeHoldModeEnabled(); }

/** Get whether switching to autonomous mode from the ground is enabled. */
bool canStartAutonomousModeGround() {
    return isAutonomousGroundEnabled() && biasManager.getThrustBias() < 0.03;
}

/** Get whether switching to autonomous mode from the air is enabled. */
bool canStartAutonomousModeAir() {
    return isAutonomousAirEnabled() && biasManager.getThrustBias() >= 0.03;
}

/** Get the next navigation target during TEST_NAVIGATION mode. */
Position getNextNavigationTestTarget() {
    navigationTargetIndex++;
    if (navigationTargetIndex == NUM_NAVIGATION_TARGETS)
        navigationTargetIndex = 0;
    return NAVIGATION_TARGETS[navigationTargetIndex];
}

/** Get whether switching to altitude mode is enabled. */
bool isAltitudeHoldModeEnabled() {
    return TEST_MODE == TestMode::TEST_ALTITUDE_HOLD ||       //
           TEST_MODE == TestMode::TEST_LOITERING ||           //
           TEST_MODE == TestMode::TEST_NAVIGATION ||          //
           TEST_MODE == TestMode::TEST_LANDING ||             //
           TEST_MODE == TestMode::TEST_QR_NAVIGATION_LOST ||  //
           TEST_MODE == TestMode::TEST_PRETAKEOFF ||          //
           TEST_MODE == TestMode::TEST_TAKEOFF ||             //
           TEST_MODE == TestMode::DEMO;                       //
}

/** Get whether autonomous mode can be activated from the air. */
bool isAutonomousAirEnabled() {
    return TEST_MODE == TestMode::TEST_LOITERING ||   //
           TEST_MODE == TestMode::TEST_NAVIGATION ||  //
           TEST_MODE == TestMode::TEST_NAVIGATION ||  //
           TEST_MODE == TestMode::TEST_LANDING ||     //
           TEST_MODE == TestMode::DEMO;
}

/** Get whether autonomous mode can be activated from the ground. */
bool isAutonomousGroundEnabled() {
    return TEST_MODE == TestMode::TEST_PRETAKEOFF ||  //
           TEST_MODE == TestMode::TEST_TAKEOFF ||     //
           TEST_MODE == TestMode::DEMO;
}

/** Get whether the drone can switch from LOITERING to CONVERGING/NAVIGATING. */
bool isNavigationEnabled() {
    return isNavigationEnabledQRCodes() || isNavigationEnabledTestTargets();
}

/**
 * Get whether the drone can switch from LOITERING to CONVERGING/NAVIGATING,
 * using QR codes to navigate.
 */
bool isNavigationEnabledQRCodes() {
    return TEST_MODE == TestMode::TEST_QR_NAVIGATION_LOST ||  //
           TEST_MODE == TestMode::DEMO;
}

/**
 * Get whether the drone can switch from LOITERING to CONVERGING/NAVIGATING,
 * using prespecified test targets to navigate.
 */
bool isNavigationEnabledTestTargets() {
    return TEST_MODE == TestMode::TEST_NAVIGATION;
}

/** Get whether the drone is able to land. */
bool isLandingEnabled() {
    return TEST_MODE == TestMode::TEST_LANDING ||  //
           TEST_MODE == TestMode::DEMO;  // Remove this if you can't land!
}

/** Get whether the drone should switch from LOITERING to LANDING. */
bool shouldLandAfterLoitering() { return TEST_MODE == TestMode::TEST_LANDING; }

/** Get whether the drone should loiter indefinitely after taking off. */
bool shouldLoiterIndefinitelyAfterTakeoff() {
    return TEST_MODE == TestMode::TEST_TAKEOFF;
}

/**
 * Get whether the spiral search will be tested after reading the first QR code.
 */
bool shouldTestQRSearch() {
    return TEST_MODE == TestMode::TEST_QR_NAVIGATION_LOST;
}
