// Original: BareMetal/src/sonar/sonar.c
#include "Sonar.hpp"
#include "HardwareConstants.hpp"
#include "MedianFilter.hpp"

// TODO: append author
/*******************************************************************************
 *   Sonar source file
 *
 *   This file contains all methods used to read inputs from the sonar
 *   and to apply a median filter and a peak filter on these measurements.
 *   author: w. devries, p. coppens
 ******************************************************************************/


/** Last 15 raw measurements of the sonar. */
real_t measurements[Sonar::MAX_MF_LENGTH];

/** Latest measurement of the sonar after median/peak filters. */
real_t filteredSonarMeasurement;

/** 
 * Counter to keep track of the number of large measurement jumps ( > 50cm ), 
 * used in peak filter.
 */
int jumpCounter = 0;

bool readSonar() {

    // Latest raw sonar value.
    static real_t newSonarRaw;

    // Previous raw sonar value, initialize to -1 when function is first called.
    static real_t oldSonarRaw = -1;

    // TODO: seriously?
    // Check if there is a new measurement available.
    newSonarRaw = (real_t) Xil_In32(SONAR_REG) / (CLK_MEASURE * PWM_TO_HEIGHT);
    if (fabs(newSonarRaw - oldSonarRaw) <= 0.00000001)
        return false;

    // Save the new measurement.
    oldSonarRaw = newSonarRaw;

    // Add the measurement to the median filter.
    addMFMeasurement(measurements, newSonarRaw);
    newSonarRaw = getMedian(MF_BUFFER_SIZE_SMALL);

    // Apply peak filter.
    real_t diff = fabs(newSonarRaw - filteredSonarMeasurement);
    if (diff > MAX_JUMP && jumpCounter < MAX_JUMP_COUNT) {
        jumpCounter++;
    } else {
        filteredSonarMeasurement = sonar_new;
        jumpCounter = 0;
    }

    return true;
}

real_t getFilteredSonarMeasurement() {
    return filteredSonarMeasurement;
}

real_t getFilteredSonarMeasurementAccurate() {
    return getMedian(measurements, MAX_MF_LENGTH, MAX_MF_LENGTH);
}

void initSonar() {

    /* Read the current sonar raw measurement. */
    readSonar();

    /**
     * Fill the sonar measurement buffer with the current measurement value so
     * that the median filter functions properly.
     */
    initMF(measurements, MAX_MF_LENGTH, getSonarMeasurement());
}
