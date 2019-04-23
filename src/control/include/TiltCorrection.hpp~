#pragma once

#include <Quaternion.hpp>

/// @see    eagle-control-slides.pdf pp.178-181
ColVector<3> getCorrectedPositionFull(ColVector<2> rawPosition,
                                      real_t rawHeight, Quaternion orientation);

/// Correct the measured location
ColVector<2> getCorrectedPosition(ColVector<2> locationMeasurement,
                                  real_t heightMeasurement, Quaternion q);

// TODO: add getCorrectedHeight() for altitude
