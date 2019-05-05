// Original: BareMetal/src/sonar/sonar.h
#include "../../../../src/utilities/include/real_t.h"
#pragma once

// TODO: append author
/*******************************************************************************
*   Sonar header file
*   This file contains all methods used to read inputs from the sonar
*   and to apply a median filter and a peak filter on these measurements.
*   author: w. devries, p. coppens
*******************************************************************************/

/**
 * Read the raw measurement from the sonar, apply the median/peak filters and
 * store the result locally.
 * 
 * @return  true
 *          If the sonar has a new measurement.
 * @return  false
 *          Otherwise.
 */
bool readSonar();

/**
 * Returns the latest filtered sonar measurement.
 */
real_t getFilteredSonarMeasurement();

/**
 * Calculates the median of the entire sonar measurement buffer.
 * 
 * @return   Median of entire sonar measurement buffer.
 */
real_t getFilteredSonarMeasurementAccurate();

/**
 * Initialise the median filter of the sonar.
 */
void initSonar();
