// Original: BareMetal/src/utils/median_filter.h

/*******************************************************************************
*   Median filter header file
*
*   This file contains an implementation of a median filter with dynamic resizing.
*   This median filter is used in sonar.cpp to filter the raw sonar measurements.
*   author: p. coppens
*******************************************************************************/

/**
 * Initialize the measurement buffer by filling it with copies of the same
 * measurement value.
 * 
 * @param   measurements
 *          Array of measurements
 * @param   size
 *          Size of array
 * @param   measurement
 *          Element to fill the buffer with
 */
void initMF(float* measurements, int size, float measurement);

/**
 * Add an element to the measurement buffer.
 * 
 * @param   measurements
 *          Array of measurements
 * @param   size
 *          Size of array
 * @param   measurement
 *          Element to add
 */
void addMFMeasurement(float* measurements, int size, float measurement);

/**
 * Get the median of the last bufferSize values currently stored in the 
 * measurement buffer.
 * 
 * @param   measurements
 *          Array of measurements
 * @param   size
 *          Size of array
 * @param   bufferSize
 *          The amount of elements that will be sorted to find the median
 */
float getMedian(float* measurements, int size, int bufferSize);
