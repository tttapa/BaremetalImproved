#include <RCValues.hpp>

/** Most recent RC values. */
static RCInput rcInput;

FlightMode getFlightMode() { return rcInput.flightMode; }

float getPitch() { return rcInput.pitch; }

float getRoll() { return rcInput.roll; }

RCInput getRCInput() { return rcInput; }

float getThrottle() { return rcInput.throttle; }

float getTuner() { return rcInput.tuner; }

WPTMode getWPTMode() { return rcInput.wptMode; }

float getYaw() { return rcInput.yaw; }

void setRCInput(RCInput values) { rcInput = values; }
