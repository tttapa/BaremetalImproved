#include <BaremetalCommunicationDef.hpp>
#include <real_t.h>

/**
 * Struct containing the two different modes or statuses of the Wireless Power
 * Transfer: OFF and ON.
 */
enum WPTMode {
    OFF = 0,  ///< Wireless Power Transfer is turned off.
    ON  = 1,  ///< Wireless Power Transfer is turned on.
};

/**
 * Struct containing the values from the RC transmitter. This includes the
 * value of the throttle, roll, pitch and yaw, which range from 0 to 1. It
 * also contains the value of the tuner knob, which ranges from -0.5 to +0.5.
 * Lastly there are switches for the flight mode and the wireless power
 * transfer. These are represented by their respective enumerations.
 */
struct RCInput {
    float throttle;         ///< Value of the RC throttle in [0,1].
    float roll;             ///< Value of the RC roll in [0,1].
    float pitch;            ///< Value of the RC pitch in [0,1].
    float yaw;              ///< Value of the RC yaw in [0,1].
    float tuner;            ///< Value of the RC tuner knob in [-0.5,+0.5].
    FlightMode flightMode;  ///< Value of the RC flight mode (as a FlightMode).
    WPTMode wptMode;        ///< Value of the RC WPT mode (as a WPTMode).
};

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
