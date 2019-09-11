#include <MiscInstances.hpp>
#include <TestMode.hpp>

/** Current test mode is MANUAL. */
static TestMode TEST_MODE = TestMode::TEST_NAVIGATION;

// /** The drone will cycle through 7 targets during the TEST_NAVIGATION mode. */
// static constexpr int NUM_NAVIGATION_TARGETS = 7;
// 
// /** Points to cycle through when during the TEST_NAVIGATION mode. */
// static const Position NAVIGATION_TARGETS[NUM_NAVIGATION_TARGETS] = {
//     Position{0.0, 0.0},    //
//     Position{1.0, 0.0},    //
//     Position{1.0, 1.0},    //
//     Position{-1.0, 1.0},   //
//     Position{-1.0, -1.0},  //
//     Position{1.0, -1.0},   //
//     Position{1.0, 0.0},    //
// };

//***** SUMMER EDIT: cycle starting target, right t
/** The drone will cycle through 2 targets during the TEST_NAVIGATION mode. */
static constexpr int NUM_NAVIGATION_TARGETS = 2;

// TODO: THESE ARE SHIFTS! NOT TARGETS!
/** Shifts to cycle through when during the TEST_NAVIGATION mode. */
static const Position NAVIGATION_TARGETS[NUM_NAVIGATION_TARGETS] = {
    Position{5.0*BLOCKS_TO_METERS, 0.0},    //
    Position{-5.0*BLOCKS_TO_METERS, 0.0},
};


/** Current navigation target index. */
static int navigationTargetIndex = 0;

/** The threshold for initialization in the air is a thrust bias of 0.30. */
static constexpr float THRUST_BIAS_THRESHOLD = 0.30;

/** Get the drone's test mode. */
TestMode getTestMode() { return TEST_MODE; }

void setTestMode(TestMode testMode) {TEST_MODE = testMode;}

/** Get whether switching to altitude mode is enabled. */
bool canStartAltitudeHoldMode() { return isAltitudeHoldModeEnabled(); }

//***** SUMMER EDIT: this is buggy! only allowing air autonomous now *****//
/** Get whether switching to autonomous mode is enabled. */
bool canStartAutonomousMode() {
    return canStartAutonomousModeAir() || canStartAutonomousModeGround();
}

/** Get whether switching to autonomous mode from the air is enabled. */
bool canStartAutonomousModeAir() {
    return true;
//    return isAutonomousAirEnabled() &&
//           biasManager.getThrustBias() >= THRUST_BIAS_THRESHOLD;
}

/** Get whether switching to autonomous mode from the ground is enabled. */
bool canStartAutonomousModeGround() {
    return false;
//    return isAutonomousGroundEnabled() &&
//           biasManager.getThrustBias() < THRUST_BIAS_THRESHOLD;
}

/** Get the next navigation target during TEST_NAVIGATION mode. */
Position getNextNavigationTestTarget() {
    //***** SUMMER EDIT: shifts instead of targets
    Position shift = NAVIGATION_TARGETS[navigationTargetIndex];
    navigationTargetIndex++;
    if (navigationTargetIndex == NUM_NAVIGATION_TARGETS)
        navigationTargetIndex = 0;
    return shift;
}

/** Get whether switching to altitude mode is enabled. */
bool isAltitudeHoldModeEnabled() {
    return TEST_MODE == TestMode::TEST_ALTITUDE_HOLD ||  //
           TEST_MODE == TestMode::TEST_LOITERING ||      //
           TEST_MODE == TestMode::TEST_NAVIGATION ||     //
           TEST_MODE == TestMode::TEST_LANDING ||
           TEST_MODE == TestMode::TEST_QR_WALKING ||          //
           TEST_MODE == TestMode::TEST_QR_NAVIGATION_LOST ||  //
           TEST_MODE == TestMode::TEST_PRETAKEOFF ||          //
           TEST_MODE == TestMode::TEST_TAKEOFF ||             //
           TEST_MODE == TestMode::DEMO;                       //
}

/** Get whether autonomous mode is enabled. */
bool isAutonomousModeEnabled() {
    return isAutonomousAirEnabled() || isAutonomousGroundEnabled();
}

/** Get whether autonomous mode can be activated from the air. */
bool isAutonomousAirEnabled() {
    return TEST_MODE == TestMode::TEST_LOITERING ||           //
           TEST_MODE == TestMode::TEST_NAVIGATION ||          //
           TEST_MODE == TestMode::TEST_LANDING ||             //
           TEST_MODE == TestMode::TEST_QR_WALKING ||          //
           TEST_MODE == TestMode::TEST_QR_NAVIGATION_LOST ||  //
           TEST_MODE == TestMode::DEMO;
}

/** Get whether autonomous mode can be activated from the ground. */
bool isAutonomousGroundEnabled() {
    return false;
    // *** SUMMER DEMO: never initialize autonomous ground
    //return TEST_MODE == TestMode::TEST_PRETAKEOFF ||  //
    //       TEST_MODE == TestMode::TEST_TAKEOFF ||     //
    //       TEST_MODE == TestMode::DEMO;
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
    return TEST_MODE == TestMode::TEST_QR_WALKING ||          //
           TEST_MODE == TestMode::TEST_QR_NAVIGATION_LOST ||  //
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
    return TEST_MODE == TestMode::TEST_NAVIGATION ||
           TEST_MODE == TestMode::TEST_LANDING ||             //
           TEST_MODE == TestMode::TEST_QR_WALKING ||          //
           TEST_MODE == TestMode::TEST_QR_NAVIGATION_LOST ||  //
           TEST_MODE == TestMode::DEMO;  // Remove this if you can't land!
}

/** Get whether the drone should loiter indefinitely after taking off. */
bool shouldLoiterIndefinitelyAfterTakeoff() {
    return TEST_MODE == TestMode::TEST_TAKEOFF;
}
/** Get whether the drone should loiter indefinitely with air initialization. */
bool shouldLoiterIndefinitelyWithInitAir() {
    return TEST_MODE == TestMode::TEST_LOITERING;
}

/**
 * Get whether the loitering timer should be shortened. This is only the case
 * during TEST_QR_WALKING mode.
 */
bool shouldLoiteringTimerBeShortened() {
    return TEST_MODE == TestMode::TEST_QR_WALKING;
}

/** Get the whether the drone should take off after pretakeoff. */
bool shouldTakeOffAfterPreTakeoff() {
    return TEST_MODE == TestMode::TEST_TAKEOFF || TEST_MODE == TestMode::DEMO;
}

/**
 * Get whether the spiral search will be tested after reading the first QR code.
 */
bool shouldTestQRSearch() {
    return TEST_MODE == TestMode::TEST_QR_NAVIGATION_LOST;
}
