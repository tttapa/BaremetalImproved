// Original: BareMetal/src/control/timers.h (updates autonomous FSM + returns AutonomousOutput)
#include <Altitude.hpp>
#include <Position.hpp>

/**
 * Output of the autonomous control system, which consists of a reference
 * position, reference height, whether the altitude controller should be
 * bypassed and which common thrust should be used if it is bypassed.
 */
struct AutonomousOutput {
    PositionReference referencePosition;
    AltitudeReference referenceHeight;
    bool bypassAltitudeController;
    real_t commonThrust;
};

/**
 * 
 */
AutonomousOutput updateAutonomousFSM();
