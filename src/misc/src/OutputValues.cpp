#include <OutputValues.hpp>

/** Current value of the common thrust. */
real_t commonThrust;

/** Current duty cycles sent to the 4 ESCs. */
MotorDutyCycles dutyCycles;

real_t getCommonThrust() { return commonThrust; }

MotorDutyCycles getDutyCycles() { return dutyCycles; }

void setCommonThrust(real_t value) { commonThrust = value; }

void setDutyCycles(MotorDutyCycles value) { dutyCycles = value; }
