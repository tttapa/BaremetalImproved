// Original: BareMetal/src/control/altitude.h
#include <real_t.h>

struct AltitudeReference {
    real_t ref;  // Height (m)
};

struct AltitudeState {
    real_t nt;  // Common motor marginal angular velocity (rad/s)
    real_t z;   // Height (m)
    real_t vz;  // Velocity (m/s)
};

struct AltitudeControlSignal {
    real_t ut;  // Common motor marginal signal (/)
};

struct AltitudeIntegralAction {
    real_t y_int;
};

struct AltitudeMeasurement {
	real_t y;
};

class Altitude {

  private:
    AltitudeReference z;
    AltitudeState x_hat;
    AltitudeControlSignal u;
    AltitudeIntegralAction y_int;
	AltitudeMeasurement y;
    void clampAltitudeControllerOutput(AltitudeControlSignal,
                                       AltitudeIntegralAction);
	//void updateReference();
    real_t ut_clamp;
	real_t rc_throttle_increase_threshold;
	real_t rc_throttle_decrease_threshold;
	real_t sonar_hz;
	real_t rc_throttle_max_cm_per_second;
	real_t z_min;
	real_t z_max;

  public:
    void updateController();
    void updateReference();
    void initializeController();
};