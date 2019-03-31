// Original: BareMetal/src/control/attitude.h
#include <quaternion.h>

struct AttitudeReference {
    Quat32 q;   // Orientation
};

struct AttitudeState {
    Quat32 q;   // Orientation
    float wx;   // X angular velocity (rad/s)
    float wy;   // Y angular velocity (rad/s)
    float wz;   // Z angular velocity (rad/s)
    float nx;   // X motor angular velocity (rad/s)
    float ny;   // Y motor angular velocity (rad/s)
    float nz;   // Z motor angular velocity (rad/s)
};

struct AttitudeControlSignal {
    float ux;   // X motor signal (/)
    float uy;   // Y motor signal (/)
    float uz;   // Z motor signal (/)
};