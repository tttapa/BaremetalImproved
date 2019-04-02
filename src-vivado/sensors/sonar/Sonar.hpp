// Original: BareMetal/src/sonar/sonar.h

// TODO: append author
/*******************************************************************************
*   Sonar header file
*   This file contains all methods used to read inputs from the sonar
*   and to apply a median filter and a peak filter on these measurements.
*   author: w. devries, p. coppens
*******************************************************************************/

// TODO: why static?

/**
 * Read the raw measurement from the sonar, apply the median/peak filters and
 * store the result locally.
 * 
 * @return  true
 *          if the sonar has a new measurement.
 * @return  false
 *          otherwise
 */
static bool readSonar();

/**
 * Returns the latest filtered sonar measurement.
 */
static real_t geFilteredtSonarMeasurement();

/**
 * Calculates the median of the entire sonar measurement buffer.
 * 
 * @return   Median of entire sonar measurement buffer.
 */
static real_t getFilteredSonarMeasurementAccurate();

/**
 * Initialise the median filter of the sonar.
 */
void initSonar();
