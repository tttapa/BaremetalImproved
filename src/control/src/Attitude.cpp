// Original: BareMetal/src/control/attitude.c

#include <Attitude.hpp>

void AttitudeController::updateObserver(AttitudeMeasurement measurement) {

    updateObserverCodegen(AttitudeController::stateEstimate,
                          AttitudeController::controlSignal,
                          measurement,
                          getDroneConfiguration());

}

AttitudeControlSignal AttitudeController::updateControlSignal(AttitudeReference reference) {

    // Calculate u_k (unclamped)
    updateControlSignalCodegen(AttitudeController::stateEstimate,
                               reference,
                               AttitudeController::controlSignal,
                               AttitudeController::integralWindup,
                               getDroneConfiguration());

    // Clamp u_k
    clampAttitudeControllerOutput(AttitudeController::controlSignal,
                                  getRCThrottle());

    return AttitudeController::controlSignal;
}

void AttitudeController::clampAttitudeControllerOutput(AttitudeControlSignal u,
                                                       real_t thrust) {
    // If definition is not negative, then clamp
    if (0 <= AttitudeController::uzClamp) {
        if (u.uz > AttitudeController::uzClamp)
            u.uz = AttitudeController::uzClamp;
        if (-u.uz > AttitudeController::uzClamp)
            u.uz = -AttitudeController::uzClamp;
    }

    //TODO: thrust clamp in de afstandsbediening zelf doen.

    // Clamp [ux;uy;uz] such that for all motor inputs vi: 0 <= vi <= 1.
    // TODO: divide by e = epsilon + 1?
    float other_max;
    float other_actual;
    other_max    = 1 - fabs(thrust);
    other_actual = fabs(AttitudeController::controlSignal.ux) +
                   fabs(AttitudeController::controlSignal.uy) +
                   fabs(AttitudeController::controlSignal.uz);
    if (other_actual > other_max) {
        controlSignal.ux =
            other_max / other_actual * AttitudeController::controlSignal.ux;
        AttitudeController::controlSignal.uy =
            other_max / other_actual * AttitudeController::controlSignal.uy;
        AttitudeController::controlSignal.uz =
            other_max / other_actual * AttitudeController::controlSignal.uz;
        other_actual = fabs(AttitudeController::controlSignal.ux) +
                       fabs(AttitudeController::controlSignal.uy) +
                       fabs(AttitudeController::controlSignal.uz);
    }
    if (other_actual > thrust) {
        AttitudeController::controlSignal.ux =
            thrust / other_actual * AttitudeController::controlSignal.ux;
        AttitudeController::controlSignal.uy =
            thrust / other_actual * AttitudeController::controlSignal.uy;
        AttitudeController::controlSignal.uz =
            thrust / other_actual * AttitudeController::controlSignal.uz;
    }
}

void AttitudeController::initializeController() {

    // reset Attitude stateEstimate
    AttitudeController::stateEstimate = {};
    // reset Attitude controlSignal
    AttitudeController::controlSignal = {};
    // reset Attitude integralWindup
    AttitudeController::integralWindup = {};
    // reset Attitude reference
    AttitudeController::reference = {};

    //TODO: dit nog steeds hier?
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
void AttitudeController::idleController() { PWMoutput(0, 0, 0, 0); }


// TODO: 
MotorDutyCycles transformAttitudeControlSignal(AttitudeControlSignal controlSignal, real_t commonThrust) {
    return MotorDutyCycles {
        commonThrust + controlSignal.ux + controlSignal.uy - controlSignal.uz,
        commonThrust + controlSignal.ux - controlSignal.uy + controlSignal.uz,
        commonThrust - controlSignal.ux + controlSignal.uy + controlSignal.uz,
        commonThrust - controlSignal.ux - controlSignal.uy - controlSignal.uz
    };
}
