#include <MiscInstances.hpp>
#include <TestMode.hpp>

/** Current test mode is MANUAL. */
static const TestMode TEST_MODE = TestMode::TEST_LOITERING;

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

/** Get whether switching to altitude mode is enabled. */
bool isAltitudeHoldModeEnabled() {
    return TEST_MODE == TestMode::TEST_ALTITUDE_HOLD ||  //
           TEST_MODE == TestMode::TEST_LOITERING ||      //
           TEST_MODE == TestMode::TEST_NAVIGATING ||     //
           TEST_MODE == TestMode::TEST_LANDING ||        //
           TEST_MODE == TestMode::TEST_TAKEOFF ||        //
           TEST_MODE == TestMode::DEMO;             //
}

/** Get whether autonomous mode can be activated from the air. */
bool isAutonomousAirEnabled() {
    return TEST_MODE == TestMode::TEST_LOITERING ||   //
           TEST_MODE == TestMode::TEST_NAVIGATING ||  //
           TEST_MODE == TestMode::TEST_NAVIGATING ||  //
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
bool isNavigatingEnabled() {
    return TEST_MODE == TestMode::TEST_NAVIGATING ||  //
           TEST_MODE == TestMode::DEMO;
}

/** Get whether the drone is able to land. */
bool isLandingEnabled() {
    return TEST_MODE == TestMode::TEST_LANDING ||  //
           TEST_MODE == TestMode::DEMO;  // Remove this if you can't land!
}

/** Get whether the drone should switch from LOITERING to LANDING. */
bool shouldLandAfterLoitering() { return TEST_MODE == TestMode::TEST_LANDING; }
