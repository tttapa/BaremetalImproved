#include "../../../src-vivado/sensors/rc/include/RC.hpp"
#include <real_t.h>

void correctDronePosition(real_t correctionX, real_t correctionY);

real_t getHoveringThrust();


real_t getRCThrottle();
real_t getRCRoll();
real_t getRCPitch();
real_t getRCYaw();
real_t getRCTuner();
int getRCMode();
int getRCInductive();

int readQRState();
real_t readQRTargetX();
real_t readQRTargetY();

void setRCInput(RCInput input);

void writeQRState(int qrState);




