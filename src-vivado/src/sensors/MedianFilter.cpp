#include "MedianFilter.hpp"
#include "math.h"

void initMF(float *measurements, int size, float measurement) {
    /* Fill the array with the given measurement value. */
    for (int i = 0; i < size; i++)
        measurements[i] = measurement;
}

void addMFMeasurement(float *measurements, int size, float measurement) {
    /* Shift all of the previous measurements (dropping the oldest one). */
    for (int i = 0; i < size - 1; i++)
        measurements[i] = measurements[i + 1];

    /* Add the new measurement. */
    measurements[size - 1] = measurement;
}

float getMedian(float *measurements, int size, int bufferSize) {

    /* We only need to order the first half of the window to determine the
       median. */
    int sortCount = ceil((bufferSize + 1) / 2);

    /* Initialize the window and fill it with the last bufferSize elements. */
    float window[bufferSize];
    for (int i = 0; i < bufferSize; i++)
        window[i] = measurements[size - 1 - i];

    /* Sort window from smallest to largest. We can stop as soon as we've found
       the median value (index sortCount). */
    for (int i = 0; i < sortCount; i++) {

        /* Find the minimum in window[i:end]. */
        int min = i;
        for (int k = i + 1; k < bufferSize; k++) {
            if (window[k] < window[min])
                min = k;
        }

        /* Swap window[i] with window[min]. */
        const float temp = window[i];
        window[i]        = window[min];
        window[min]      = temp;
    }

    /* Return the median value. */
    return window[sortCount - 1];
}
