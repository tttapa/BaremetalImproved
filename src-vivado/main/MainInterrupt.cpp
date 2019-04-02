#include "MainInterrupt.hpp"
#include "../sensors/ahrs/AHRS.hpp"
#include "../sensors/imu/IMU.hpp"
#include "../time/Time.hpp"
#include <MainFSM.hpp>  // Update control system

void update() {

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
        initAHRS();
        isAHRSInitialized = true;
    }
    // Phase 3: main operation
    else {
        updateMainFSM();
    }
}
