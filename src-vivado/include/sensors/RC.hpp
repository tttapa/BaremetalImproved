#pragma once

/* Includes from src. */
#include <LoggerStructs.hpp>  ///< RCInput

/**
 * Read the RC voltages from the registers.
 * 
 * @return  A struct containing values for the RC throttle which is in [0,1] and
 *          for the roll, pitch, yaw and tuner knob which are in [-1,1]. Also
 *          there are enumerations for the flight mode and the WPT mode.
 */
RCInput readRC();
