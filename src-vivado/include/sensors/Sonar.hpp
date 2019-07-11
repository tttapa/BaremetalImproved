#pragma once

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
float getFilteredSonarMeasurement();

/**
 * Calculates the median of the entire sonar measurement buffer.
 * 
 * @return   Median of entire sonar measurement buffer.
 */
float getFilteredSonarMeasurementAccurate();

/**
 * Returns the latest unfiltered sonar measurement.
 */
float getUnfilteredSonarMeasurement();

/**
 * Initialise the median filter of the sonar.
 */
void initSonar();
