// Original: BareMetal/src/control/altitude.h
struct AltitudeReference {
    float z;    // Height (m)
};

struct AltitudeState {
    float nt;   // Common motor marginal angular velocity (rad/s)
    float z;    // Height (m)
    float vz;   // Velocity (m/s)
};

struct AltitudeControlSignal {
    float ut;   // Common motor marginal signal (/)
}