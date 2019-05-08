#include <OutputValues.hpp>

/** Current value of the common thrust. */
real_t commonThrust;

/** Current duty cycles sent to the 4 ESCs. */
MotorSignals motorSignals;

real_t getCommonThrust() { return commonThrust; }

MotorSignals getMotorSignals() { return motorSignals; }

void setCommonThrust(real_t value) { commonThrust = value; }

void setMotorSignals(MotorSignals value) { motorSignals = value; }
