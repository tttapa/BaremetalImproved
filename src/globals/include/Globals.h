#include "../../../src-vivado/sensors/rc/include/RC.hpp"
#include <real_t.h>

void setRCInput(struct RCInput input);

real_t getRCThrottle();
real_t getRCRoll();
real_t getRCPitch();
real_t getRCYaw();
real_t getRCTuner();

int getRCMode();
int getRCInductive();



