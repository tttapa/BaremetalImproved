#pragma once

#include <Quaternion.hpp>
#include <real_t.h>

/*
/// @see    eagle-control-slides.pdf pp.178-181
ColVector<3> getCorrectedPositionFull(ColVector<2> rawPosition,
                                      real_t rawHeight, Quaternion orientation);
*/

/// Correct the measured location
ColVector<2> getCorrectedPosition(ColVector<2> locationMeasurement,
                                  real_t heightMeasurement, Quaternion q);

/// Correct the measured height
real_t getCorrectedHeight(real_t heightMeasurement, Quaternion q);
