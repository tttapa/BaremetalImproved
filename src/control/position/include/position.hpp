// Original: BareMetal/src/control/navigation.h
struct PositionReference {
    float x;        // X position (m)
    float y;        // Y position (m)
};

struct PositionState {
    float q1;       // Orientation q1 component (/)
    float q2;       // Orientation q2 component (/)
    float x;        // X position (m)
    float y;        // Y position (m)
    float vx;       // X velocity (m/s)
    float vy;       // Y velocity (m/s)
};

struct PositionControlSignal {
    float q1ref;    // Reference orientation q1 component (/)
    float q2ref;    // Reference orientation q2 component (/)
};
