#include <real_t.h>
#include <RCTypes.hpp>


real_t getRCThrottle();
real_t getRCRoll();
real_t getRCPitch();
real_t getRCYaw();
real_t getRCTuner();
FlightMode getRCFlightMode();
WPTMode getRCWPTMode();

int readQRState();
real_t readQRTargetX();
real_t readQRTargetY();

void setRCInput(RCInput input);

void writeQRState(int qrState);




