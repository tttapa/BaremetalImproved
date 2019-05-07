#include <Globals.hpp>

static RCInput currentRCInput;

real_t getRCThrottle() { return currentRCInput.throttle; }

real_t getRCPitch() { return currentRCInput.pitch; }

real_t getRCRoll() { return currentRCInput.roll; }

real_t getRCYaw() { return currentRCInput.yaw; }

real_t getRCTuner() { return currentRCInput.tuner; }

FlightMode getRCFlightMode() { return currentRCInput.flightMode; }

WPTMode getRCWPTMode() { return currentRCInput.wptMode; }

// TODO: read from shared memory
QRFSMState readQRState() { return QRFSMState::IDLE; }
real_t readQRTargetX() { return 0.5; }
real_t readQRTargetY() {return 0.5; }

void setRCInput(RCInput input) { currentRCInput = input; }

// TODO: write to shared memory
void writeQRState(QRFSMState qrState) { (void)qrState; }
