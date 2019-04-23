#include "../include/Globals.h"

static struct RCInput RC;

real_t getRCThrottle() {
    return RC.throttle;
}

real_t getRCPitch() {
    return RC.pitch;
}

real_t getRCRoll() {
    return RC.roll;
}

real_t getRCYaw() {
    return RC.yaw;
}

real_t getRCTuner() {
    return RC.tuner;
}

int getRCMode() {
    return RC.mode;
}

int getRCInductive() {
    return RC.inductive;
}



