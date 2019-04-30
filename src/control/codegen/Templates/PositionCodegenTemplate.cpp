#include <Position.hpp>
#include <math.h>   /* fabs, copysign */
#include <string.h> /* memcpy */

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
PositionControlSignal
codegenControlSignal(PositionState stateEstimate, PositionReference reference,
                     PositionIntegralWindup integralWindup,
                     int droneConfiguration) {

    PositionControlSignal controlSignal;

    /* Calculate controller output. */
    switch (droneConfiguration) {
        case 1:
            controlSignal.q1ref = $c1$u0;
            controlSignal.q2ref = $c1$u1;
            break;
        case 2:
            controlSignal.q1ref = $c2$u0;
            controlSignal.q2ref = $c2$u1;
            break;
        case 3:
            controlSignal.q1ref = $c3$u0;
            controlSignal.q2ref = $c3$u1;
            break;
        case 4:
            controlSignal.q1ref = $c4$u0;
            controlSignal.q2ref = $c4$u1;
            break;
        default: controlSignal = {};
    }
}

/* Don't use integral action if tunerValue < 0.0. */
//if(tunerValue < 0.0)
//	y_int_max = 0.0;
// (void)tunerValue;

PositionIntegralWindup
codegenIntegralWindup(PositionIntegralWindup integralWindup,
                      PositionReference reference, PositionState stateEstimate,
                      int droneConfiguration) {

    real_t maxIntegralWindup;

    switch (configuration) {
        case 1: maxIntegralWindup = $c1$maxWindup; break;
        case 2: maxIntegralWindup = $c2$maxWindup; break;
        case 3: maxIntegralWindup = $c3$maxWindup; break;
        case 4: maxIntegralWindup = $c4$maxWindup; break;
        default: maxIntegralWindup = 0.0;
    }

    integralWindup.x += $int0;
    integralWindup.y += $int1;
    if (fabs(integralWindup.x) > maxIntegralWindup)
        integralWindup.x = copysign(maxIntegralWindup, integralWindup.x);
    if (fabs(integralWindup.y) > maxIntegralWindup)
        integralWindup.y = copysign(maxIntegralWindup, integralWindup.y);
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template.
 */
PositionState codegenCurrentStateEstimate(PositionState stateEstimate,
                                          PositionMeasurement measurement,
                                          Quaternion orientation,
                                          int droneConfiguration) {

    // TODO: this shouldn't be necessary when IMP works without errors

    // Don't fuck up the observer's velocity if IMP sends weird data

    // Calculate velocity
    float vThresholdAway    = 0.30;  // 30 cm/s away from 0: ignore
    float vThresholdTowards = 0.50;  // 40 cm/s towards 0: ignore

    //TODO: Ts ook meegeven?
    float vx = (measurement.x - stateEstimate.x) / Ts;
    float vy = (measurement.y - stateEstimate.y) / Ts;

    // X Towards
    if (fabs(vx) - fabs(stateEstimate.vx) <= 0 &&
        fabs(vx - stateEstimate.vx) < vThresholdTowards) {
        stateEstimate.vx = vx;
        // X Away
    } else if (fabs(vx) - fabs(stateEstimate.vx) >= 0 &&
               fabs(vx - stateEstimate.vx) < vThresholdAway) {
        stateEstimate.vx = vx;
    }

    // Y Towards
    if (fabs(vx) - fabs(stateEstimate.vy) <= 0 &&
        fabs(vy - stateEstimate.vy) < vThresholdTowards) {
        stateEstimate.vy = vy;
        // Y Away
    } else if (fabs(vx) - fabs(stateEstimate.vy) >= 0 &&
               fabs(vy - stateEstimate.vy) < vThresholdAway) {
        stateEstimate.vy = vy;
    }

    /*
    if(fabs(vx - x_hat[4]) < vThreshold)
        x_hat[4] = vx;
    if(fabs(vy - x_hat[5]) < vThreshold)
        x_hat[5] = vy;
    */

    // TODO: only do this if IMP works perfectly!
    /*
    // Calculate velocity
    x_hat[4] = (y[0] - x_hat[2]) / Ts;
    x_hat[5] = (y[1] - x_hat[3]) / Ts;
    */

    // Set orientation and position
    stateEstimate.q1 = orientation[1];
    stateEstimate.q2 = orientation[2];
    stateEstimate.x  = measurement.x;
    stateEstimate.y  = measurement.y;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template.
 */
/*
void updateNavigationObserver(NavigationState x_hat,
                              const_NavigationControl u,
                              const_NavigationOutput y, 
                              const_AttitudeState att_x_hat, float Ts, 
                              int configuration) {

    NavigationState x_hat_copy;
    memcpy(x_hat_copy, x_hat, sizeof(x_hat_copy));

    switch(configuration) {
        case 1:
            x_hat[0] = $c1$x0;
            x_hat[1] = $c1$x1;
            x_hat[2] = $c1$x2;
            x_hat[3] = $c1$x3;
            x_hat[4] = $c1$x4;
            x_hat[5] = $c1$x5;
            break;
        case 2:
            x_hat[0] = $c2$x0;
            x_hat[1] = $c2$x1;
            x_hat[2] = $c2$x2;
            x_hat[3] = $c2$x3;
            x_hat[4] = $c2$x4;
            x_hat[5] = $c2$x5;
            break;
        case 3:
            x_hat[0] = $c3$x0;
            x_hat[1] = $c3$x1;
            x_hat[2] = $c3$x2;
            x_hat[3] = $c3$x3;
            x_hat[4] = $c3$x4;
            x_hat[5] = $c3$x5;
            break;
        case 4:
            x_hat[0] = $c4$x0;
            x_hat[1] = $c4$x1;
            x_hat[2] = $c4$x2;
            x_hat[3] = $c4$x3;
            x_hat[4] = $c4$x4;
            x_hat[5] = $c4$x5;
            break;
    }


}
*/
