// Original: BareMetal/src/sonar/sonar.c
#include "sonar.hpp"
#include "hardware-registers.hpp"
#include "median-filter.hpp"

// TODO: append author
/*******************************************************************************
 *   Sonar source file
 *
 *   This file contains all methods used to read inputs from the sonar
 *   and to apply a median filter and a peak filter on these measurements.
 *   author: w. devries, p. coppens
 ******************************************************************************/

/** Conversion factor to meters. */
static float PWM_TO_HEIGHT = 0.005787;

/** Size of the median filter buffer during flight. */
static int MF_BUFFER_SIZE_SMALL = 5;

/** 
 * Total number of measurements stored in the sonar measurement buffer. This is 
 * the maximum median filter window size. 
 */
static int MAX_MF_LENGTH = 15;

/** 
 * Maximum amount of subsequent iterations where the peak filter is allowed to
 * discard a measurement. 
 */
static int MAX_JUMP_COUNT = 3;

/** 
 * Maximum amount of meters the height can change in 1 iteration 
 * (after applying median filter). 
 */
static float MAX_JUMP = 0.5;

/** Last 15 raw measurements of the sonar. */
static float measurements[MAX_MF_LENGTH];

/** Latest measurement of the sonar after median/peak filters. */
static float filteredSonarMeasurement;

/** 
 * Counter to keep track of the number of large measurement jumps ( > 50cm ), 
 * used in peak filter.
 */
static int jumpCounter = 0;

static bool readSonar() {

    // Latest raw sonar value.
    static float newSonarRaw;

    // Previous raw sonar value, initialize to -1 when function is first called.
    static float oldSonarRaw = -1;

    // TODO: seriously?
    // Check if there is a new measurement available.
    newSonarRaw = (float) Xil_In32(SONAR_REG) / (CLK_MEASURE * PWM_TO_HEIGHT);
    if (fabs(newSonarRaw - oldSonarRaw) <= 0.00000001)
        return false;

    // Save the new measurement.
    oldSonarRaw = newSonarRaw;

    // Add the measurement to the median filter.
    addMFMeasurement(measurements, newSonarRaw);
    newSonarRaw = getMedian(MF_BUFFER_SIZE_SMALL);

    // Apply peak filter.
    float diff = fabs(newSonarRaw - filteredSonarMeasurement);
    if (diff > MAX_JUMP && jumpCounter < MAX_JUMP_COUNT) {
        jumpCounter++;
    } else {
        filteredSonarMeasurement = sonar_new;
        jumpCounter              = 0;
    }

    return true;
}

static float getFilteredSonarMeasurement() { return filteredSonarMeasurement; }

static float getFilteredSonarMeasurementAccurate() {
    return getMedian(measurements, MAX_MF_LENGTH, MAX_MF_LENGTH);
}

static void initSonar() {
    // Read the current sonar raw measurement.
    readSonar();

    // Fill the sonar measurement buffer with the current measurement value so
    // that the median filter functions properly.
    initMF(measurements, MAX_MF_LENGTH, getSonarMeasurement());
}
