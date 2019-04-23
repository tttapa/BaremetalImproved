// Original: BareMetal/src/control/altitude.c

#include <Matrix.hpp>
#include <Quaternion.hpp>
#include <altitude.hpp>
#include <sonar.hpp>
#include <altitude-controller.h>

void Altitude::updateController() {

    // Calculate u_k (unclamped)
    getAltitudeControllerOutput(x_hat, z, u, y_int, 0, 0);
    //TODO: currentDroneConfiguration and currentRC.. nog aanpassen

    // Clamp u_k
    clampAltitudeControllerOutput(u, y_int);
}

void Altitude::clampAltitudeControllerOutput(AltitudeControlSignal u,
                                             AltitudeIntegralAction y_int) {
    if (u.ut > Altitude::ut_clamp)
        u.ut = Altitude::ut_clamp;
    else if (u.ut < -Altitude::ut_clamp)
        u.ut = -Altitude::ut_clamp;
}

void Altitude::updateReference() {
    //TODO: implement RC functions
    real_t thrust = getRCThrust();

    real_t target_increase;
    if (thrust > Altitude::rc_throttle_increase_threshold) {
        target_increase =
            (thrust - Altitude::rc_throttle_increase_threshold) *
            Altitude::rc_throttle_max_cm_per_second / 100.0 / Altitude::sonar_hz;
        z.ref += target_increase;
    } else if (thrust < Altitude::rc_throttle_decrease_threshold) {
        target_increase =
            (thrust - Altitude::rc_throttle_decrease_threshold) *
            Altitude::rc_throttle_max_cm_per_second / 100.0 / Altitude::sonar_hz;
        z.ref += target_increase;
    }
    if (z.ref < Altitude::z_min) {
        z.ref = Altitude::z_min;
    }
    if (z.ref > Altitude::z_max) {
        z.ref = Altitude::z_max;
    }
}

void Altitude::initializeController() {
    // From now on, attempt to stay at the height we were at on altitude switch 
    // (initialized only when going from manual to altitude)
	real_t pz = getFilteredSonarMeasurementAccurate();
    //TODO: correct_tilt_height nog maken
	correct_tilt_height(pz);
	z.ref = pz;
	if(z.ref < Altitude::z_min) z.ref = Altitude::z_min;
	if(z.ref > Altitude::z_max) z.ref = Altitude::z_max;
	
    x_hat = {};
    x_hat.z = pz;
	u = {};
	y_int = {};

    //TODO: rest nog aanpassen
	// Reset final descent timer
	//land_finalDescentCounter = 0;

	// INITIALIZE ALT_THRUST SO THAT WE DON'T START FALLING WHILE WAITING FOR FIRST SONAR MEASUREMENT
	//thrust_out();
}
