#pragma once
#include <Quaternion.hpp>
#include <real_t.h>

/**
 * Correct the given measurement sent by the Image Processing team using the
 * the most recent sonar measurement and orientation. For the derivation of
 * the formula, see eagle-control-slides.pdf pp.178-181.
 * 
 * @param   impMeasurement
 *          column vector with two rows, containing the x- and y-coordinate
 *          sent by the Image Processing team, to correct
 * @param   sonarMeasurment
 *          most recent sonar measurement
 * @param   orientation
 *          orientation of the drone
 * 
 * @return  a column vector with two rows, representing the corrected position
 *          of the drone (x,y).
 */
ColVector<2> getCorrectedPosition(ColVector<2> impMeasurement,
                                  real_t sonarMeasurement,
                                  Quaternion orientation);

/**
 * Correct the given sonar measurement using the most recent orientation. For
 * the derivation of the formula, see eagle-control-slides.pdf pp.178-181.
 * 
 * @param   sonarMeasurement
 *          sonar measurement to correct
 * @param   orientation
 *          orientation of the drone
 * 
 * @return  the corrected height of the drone.
 */
real_t getCorrectedHeight(real_t sonarMeasurement, Quaternion orientation);
