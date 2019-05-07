#include <RCValues.hpp>

/** Most recent RC values. */
static RCInput rcInput;

FlightMode getFlightMode() { return rcInput.flightMode; }

real_t getPitch() { return rcInput.pitch; }

real_t getRoll() { return rcInput.roll; }

real_t getThrottle() { return rcInput.throttle; }

real_t getTuner() { return rcInput.tuner; }

WPTMode getWPTMode() { return rcInput.wptMode; }

real_t getYaw() { return rcInput.yaw; }

void setRCInput(RCInput values) { rcInput = values; }
