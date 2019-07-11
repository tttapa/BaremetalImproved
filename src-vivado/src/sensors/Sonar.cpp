#include <cmath>
#include <sensors/Sonar.hpp>

/* Includes from src. */
#include <PublicHardwareConstants.hpp>  ///< SONAR_FREQUENCY

/* Includes from src-vivado. */
#include "../PrivateHardwareConstants.hpp"
#include "MedianFilter.hpp"

/* Includes from Xilinx. */
#include <xil_io.h>

#pragma region Constants
/** Address of the sonar : // TODO: what pin? */
uint32_t *const SONAR_ADDR = ((uint32_t *) XPAR_RC_1_S00_AXI_BASEADDR) + 1;

/** Conversion factor to meters. */
const float PWM_TO_HEIGHT = 0.005787;  // TODO: is this correct?

// TODO: i think this causes altitude controllers to overshoot
// TODO: 10 Hz = 250ms delay, 20 Hz = 125 ms delay
/** Size of the median filter buffer during flight. */
const int MF_BUFFER_SIZE_SMALL = 5;

/** 
 * Total number of measurements stored in the sonar measurement buffer. This is 
 * the maximum median filter window size. 
 */
const int MAX_MF_LENGTH = 15;

//***** SUMMER EDIT: jump rejection @ 15cm, 5 outliers *****//
/** 
 * Maximum amount of subsequent iterations where the peak filter is allowed to
 * discard a measurement. 
 */
//const int MAX_JUMP_COUNT = 3;
const int MAX_JUMP_COUNT = 5;

/** 
 * Maximum amount of meters the height can change in 1 iteration 
 * (after applying median filter). 
 */
//const float MAX_JUMP = 0.5;
const float MAX_JUMP = 0.15;

/** Frequency of the sonar measurements. */
const float FREQUENCY = SONAR_FREQUENCY;
#pragma endregion

/** Last 15 raw measurements of the sonar. */
float measurements[MAX_MF_LENGTH];

/** Latest measurement of the sonar after median/peak filters. */
float filteredSonarMeasurement;

/** Whether the sonar is initialized. */
bool isSonarInitialized = false;

/** 
 * Counter to keep track of the number of large measurement jumps ( > 50cm ), 
 * used in peak filter.
 */
int jumpCounter = 0;

/** Latest raw sonar value. */
static float newSonarRaw;

/**
 * Previous raw sonar value, initialize to -1 when function is first
 * called.
 */
static float oldSonarRaw = -1;

//***** SUMMER EDIT: ema to smooth out measurements instead of median filter *****//
static float sonarEMAState;

bool readSonar() {

    /* Check if there is a new measurement available. */
    newSonarRaw = (float) Xil_In32((uintptr_t) SONAR_ADDR) /
                  (CLOCK_FREQUENCY * PWM_TO_HEIGHT);
    if (fabs(newSonarRaw - oldSonarRaw) <= 0.00000001)
        return false;

    /* Save the new measurement. */
    oldSonarRaw = newSonarRaw;

    //***** SUMMER EDIT: this is reworked so median is replaced by EMA
    //                   and peak filter comes first. *****//

    /* On initialization, set EMA to initial measurement. */
    if (!isSonarInitialized) {
        sonarEMAState = newSonarRaw;
        isSonarInitialized = true;
    }

    /* Apply peak filter. */
    float diff = fabs(newSonarRaw - sonarEMAState);
    if (diff > MAX_JUMP && jumpCounter < MAX_JUMP_COUNT) {
        jumpCounter++;
        /* Don't update sonarEMAState. */
        // TODO: if there's a big jump, should we still say there's a new meas?
        //       YES, this keeps a constant measurement frequency
    } else {
    	jumpCounter = 0;

    	/* Apply EMA. */
        float alpha = 0.3;
        sonarEMAState = alpha * newSonarRaw + (1 - alpha) * sonarEMAState;
    }

    /* sonarEMAState is filtered sonar measurement. TODO: use pieter's class */
    filteredSonarMeasurement = sonarEMAState;

    return true;
}

float getFilteredSonarMeasurement() { return filteredSonarMeasurement; }

float getFilteredSonarMeasurementAccurate() {
    return getMedian(measurements, MAX_MF_LENGTH, MAX_MF_LENGTH);
}

float getUnfilteredSonarMeasurement() { return newSonarRaw; }

void initSonar() {
    /* Initialize the sonar. Fill the sonar measurement buffer with the current
       measurement value so that the median filter works properly. */
    isSonarInitialized = false;
    readSonar();
}
