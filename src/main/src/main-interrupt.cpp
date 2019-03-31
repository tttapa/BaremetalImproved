#include <Time.hpp>
#include <main-FSM.hpp>
#include <main-interrupt.hpp>
#include <IMU.hpp>
#include <AHRS.hpp>

void updateInterrupt() {
    // Keep the clock/timer up-to-date
    incrementTickCount();

    // IMU bias should be calculated before use.
    static bool isIMUCalibrated = false;
    // AHRS should calibrate with accelerometer before use.
    static bool isAHRSInitialized = false;

    // Phase 1: Calibrate IMU.
    if (!isIMUCalibrated) {
        isIMUCalibrated = calibrateIMU();
    }
    // Phase 2: Initialize AHRS. */
    else if (!isAHRSInitialized) {
        initializeAHRS();
        isAHRSInitialized = true;
    }
    // Phase 3: main operation
    else {
        updateMainFSM();
    }
}