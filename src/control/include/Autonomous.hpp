// Original: BareMetal/src/control/timers.h (updates autonomous FSM + returns AutonomousOutput)
#include <altitude.hpp>
#include <position.hpp>

struct AutonomousOutput {
    PositionReference location;
    AltitudeReference height;
    bool bypassAltitudeController;
    real_t commonThrust;
};

AutonomousOutput updateAutonomousFSM();
