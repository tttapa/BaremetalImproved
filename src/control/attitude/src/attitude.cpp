// Original: BareMetal/src/control/attitude.c

#include <attitude.hpp>

void Attitude::updateController() {

    // TODO: RC functions
    real_t thrust = getRCThrust();
    real_t roll   = getRCRoll();
    real_t pitch  = getRCPitch();
    real_t yaw    = getRCYaw();

    // Calculate u_k (unclamped)
    getAttitudeControllerOutput(Attitude::x_hat, Attitude::ref, Attitude::u,
                                Attitude::y_int, 0, 0);
    //TODO: replace zeros with currentDroneConfig, ..

    // Clamp u_k
    clampAttitudeControllerOutput(u, thrust);
}

void Attitude::clampAttitudeControllerOutput(AttitudeControlSignal u,
                                             real_t thrust) {
    // If definition is not negative, then clamp
    if (0 <= Attitude::uz_clamp) {
        if (u.uz > Attitude::uz_clamp)
            u.uz = Attitude::uz_clamp;
        if (-u.uz > Attitude::uz_clamp)
            u.uz = -Attitude::uz_clamp;
    }

    // Clamp thrust to [0,THRUST_MAX], so we can still correct attitude 
    // if the user sets the thrust to 100%.
    if (thrust < 0)
        thrust = 0;
    if (thrust > Attitude::thrust_clamp)
        thrust = Attitude::thrust_clamp;

    // Clamp [ux;uy;uz] such that for all motor inputs vi: 0 <= vi <= 1.
    // TODO: divide by e = epsilon + 1?
    float other_max;
    float other_actual;
    other_max    = 1 - fabs(thrust);
    other_actual = fabs(u.ux) + fabs(u.uy) + fabs(u.uz);
    if (other_actual > other_max) {
        u.ux         = other_max / other_actual * u.ux;
        u.uy         = other_max / other_actual * u.uy;
        u.uz         = other_max / other_actual * u.uz;
        other_actual = fabs(u.ux) + fabs(u.uy) + fabs(u.uz);
    }
    if (other_actual > thrust) {
        u.ux = thrust / other_actual * u.ux;
        u.uy = thrust / other_actual * u.uy;
        u.uz = thrust / other_actual * u.uz;
    }
}


void Attitude::initializeController() {

    // reset AttitudeObserverEstimate
    x_hat = {};
    // reset AttitudeControlSignal
    u = {};
    // reset IntegralAction
    y_int = {};
    // reset AttitudeReference
    ref = {};

    //TODO: nog niet af
    // Also reset the yaw counters here
    //att_total_dyaw_rads = 0;
    //rel_yaw = 0.0;

    // Reset gyro measurement (remove drift)
    //ahrs_orient.w = 1.0;
    //ahrs_orient.x = 0.0;
    //ahrs_orient.y = 0.0;
    //ahrs_orient.z = 0.0;
}

/**
 * This method turns of all motors by sending out PWM to all ESC's
 * with v[4] = {0, 0, 0, 0}
 */
//TODO: PWMOutput
void Attitude::idleController() { PWMoutput(0, 0, 0, 0); }


