#pragma once

/**
 * Initialize the measurement buffer by filling it with copies of the same
 * measurement value.
 * 
 * @param   measurements
 *          Array of measurements.
 * @param   size
 *          Size of array.
 * @param   measurement
 *          Element to fill the buffer with.
 */
void initMF(float *measurements, int size, float measurement);

/**
 * Add an element to the measurement buffer.
 * 
 * @param   measurements
 *          Array of measurements.
 * @param   size
 *          Size of array.
 * @param   measurement
 *          Element to add.
 */
void addMFMeasurement(float *measurements, int size, float measurement);

/**
 * Get the median of the last bufferSize values currently stored in the 
 * measurement buffer.
 * 
 * @param   measurements
 *          Array of measurements.
 * @param   size
 *          Size of array.
 * @param   bufferSize
 *          The amount of elements that will be sorted to find the median.
 */
float getMedian(float *measurements, int size, int bufferSize);
