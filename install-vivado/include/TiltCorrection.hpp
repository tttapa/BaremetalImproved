#pragma once

/* Includes from src. */
#include <Quaternion.hpp>
#include <real_t.h>
#include <Position.hpp>

/**
 * Correct the given measurement sent by the Image Processing team using the
 * the most recent sonar measurement and orientation. For the derivation of
 * the formula, see eagle-control-slides.pdf pp.178-181.
 * 
 * @param   impMeasurement
 *          Column vector with two rows, containing the x- and y-coordinate
 *          sent by the Image Processing team, to correct.
 * @param   sonarMeasurment
 *          Most recent sonar measurement.
 * @param   orientation
 *          Orientation of the drone.
 * 
 * @return  A column vector with two rows, representing the corrected position
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
 *          Sonar measurement to correct.
 * @param   orientation
 *          Orientation of the drone.
 * 
 * @return  The corrected height of the drone.
 */
real_t getCorrectedHeight(real_t sonarMeasurement, Quaternion orientation);


// TODO: documentation
ColVector<2>
getGlobalPositionEstimate(ColVector<2> correctedPositionMeasurement,
                          PositionState lastPositionEstimate, real_t Ts);
