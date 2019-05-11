#include <TestManager.hpp>

/** Current test mode is MANUAL. */
static const TestMode TEST_MODE = TestMode::MANUAL;

/** Get the drone's test mode. */
TestMode getTestMode() { return TEST_MODE; }

/** Get whether switching to altitude mode is enabled. */
bool isAltitudeHoldModeEnabled() {
    return TEST_MODE == TestMode::ALTITUDE_HOLD ||  //
           TEST_MODE == TestMode::LOITERING ||      //
           TEST_MODE == TestMode::NAVIGATING ||     //
           TEST_MODE == TestMode::LANDING ||        //
           TEST_MODE == TestMode::TAKEOFF ||        //
           TEST_MODE == TestMode::DEMO;             //
}

/** Get whether switching to autonomous mode is enabled. */
bool isAutonomousModeEnabled() {
    return TEST_MODE == TestMode::LOITERING ||      //
           TEST_MODE == TestMode::NAVIGATING ||     //
           TEST_MODE == TestMode::LANDING ||        //
           TEST_MODE == TestMode::TAKEOFF ||        //
           TEST_MODE == TestMode::DEMO;             //
}

/** Get whether autonomous mode can be activated frmo the air. */
bool isAutonomousAirEnabled() {
    
}