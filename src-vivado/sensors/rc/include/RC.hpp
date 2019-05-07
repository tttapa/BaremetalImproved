#pragma once
#include <RCValues.hpp>

/**
 * Read the RC voltages from the registers.
 * 
 * @return  A struct containing values for the RC throttle, roll, pitch, yaw,
 *          each of which is in [0,1], for the tuner knob, which is in
 *          [-0.5,+0,5], and for the flight mode in {0,1,2} and WPT mode in
 *          {0,1}.
 */
RCInput readRC();
