#pragma once
#include <RCTypes.hpp>
#include <real_t.h>

real_t getRCThrottle();
real_t getRCRoll();
real_t getRCPitch();
real_t getRCYaw();
real_t getRCTuner();
FlightMode getRCFlightMode();
WPTMode getRCWPTMode();

void setRCInput(RCInput input);

