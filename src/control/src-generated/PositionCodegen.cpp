/* Automatically generated, using 
 *
 * Configuration 1: 
 * Q = 
 *   3 0 0 0 0 0
 *   0 3 0 0 0 0
 *   0 0 9.000000e-01 0 0 0
 *   0 0 0 9.000000e-01 0 0
 *   0 0 0 0 1.500000e-02 0
 *   0 0 0 0 0 1.500000e-02
 * R = 
 *   30 0
 *   0 30
 * I = 
 *   0 -1.000000e-03
 *   1.000000e-03 0
 *
 * Configuration 2: 
 * Q = 
 *   3 0 0 0 0 0
 *   0 3 0 0 0 0
 *   0 0 9.000000e-01 0 0 0
 *   0 0 0 9.000000e-01 0 0
 *   0 0 0 0 1.500000e-02 0
 *   0 0 0 0 0 1.500000e-02
 * R = 
 *   30 0
 *   0 30
 * I = 
 *   0 -1.000000e-03
 *   1.000000e-03 0
 *
 * Configuration 3: 
 * Q = 
 *   1 0 0 0 0 0
 *   0 1 0 0 0 0
 *   0 0 3.000000e-01 0 0 0
 *   0 0 0 3.000000e-01 0 0
 *   0 0 0 0 1.000000e-03 0
 *   0 0 0 0 0 1.000000e-03
 * R = 
 *   15 0
 *   0 15
 * I = 
 *   0 -1.000000e-02
 *   1.000000e-02 0
 *
 * Configuration 4: 
 * Q = 
 *   1.000000e-02 0 0 0 0 0
 *   0 1.000000e-02 0 0 0 0
 *   0 0 3.000000e-01 0 0 0
 *   0 0 0 3.000000e-01 0 0
 *   0 0 0 0 1.000000e-03 0
 *   0 0 0 0 0 1.000000e-03
 * R = 
 *   30 0
 *   0 30
 * I = 
 *   0 -1.000000e-02
 *   1.000000e-02 0
 *
 */

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
            controlSignal.q1ref = 0.14859985000000001*stateEstimate.y - 0.79794841000000005*stateEstimate.q1 - 0.14859985000000001*reference.y + 0.16614145*stateEstimate.vy - 0.001*integralWindup.y;
            controlSignal.q2ref = 0.14859985000000001*reference.x - 0.79794841000000005*stateEstimate.q2 - 0.14859985000000001*stateEstimate.x - 0.16614145*stateEstimate.vx + 0.001*integralWindup.x;
            break;
        case 2:
            controlSignal.q1ref = 0.14859985000000001*stateEstimate.y - 0.79794841000000005*stateEstimate.q1 - 0.14859985000000001*reference.y + 0.16614145*stateEstimate.vy - 0.001*integralWindup.y;
            controlSignal.q2ref = 0.14859985000000001*reference.x - 0.79794841000000005*stateEstimate.q2 - 0.14859985000000001*stateEstimate.x - 0.16614145*stateEstimate.vx + 0.001*integralWindup.x;
            break;
        case 3:
            controlSignal.q1ref = 0.12327784*stateEstimate.y - 0.72226261999999997*stateEstimate.q1 - 0.12327784*reference.y + 0.14728732*stateEstimate.vy - 0.01*integralWindup.y;
            controlSignal.q2ref = 0.12327784*reference.x - 0.72226261999999997*stateEstimate.q2 - 0.12327784*stateEstimate.x - 0.14728732*stateEstimate.vx + 0.01*integralWindup.x;
            break;
        case 4:
            controlSignal.q1ref = 0.089303110000000005*stateEstimate.y - 0.60622721999999996*stateEstimate.q1 - 0.089303110000000005*reference.y + 0.12103102*stateEstimate.vy - 0.01*integralWindup.y;
            controlSignal.q2ref = 0.089303110000000005*reference.x - 0.60622721999999996*stateEstimate.q2 - 0.089303110000000005*stateEstimate.x - 0.12103102*stateEstimate.vx + 0.01*integralWindup.x;
            break;
        default: controlSignal = {};
    }
    return controlSignal;
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

    switch (droneConfiguration) {
        case 1: maxIntegralWindup = 10; break;
        case 2: maxIntegralWindup = 10; break;
        case 3: maxIntegralWindup = 10; break;
        case 4: maxIntegralWindup = 10; break;
        default: maxIntegralWindup = 0.0;
    }

    integralWindup.x += 0.11764705882352941*reference.x - 0.11764705882352941*stateEstimate.x;
    integralWindup.y += 0.11764705882352941*reference.y - 0.11764705882352941*stateEstimate.y;
    if (fabs(integralWindup.x) > maxIntegralWindup)
        integralWindup.x = copysign(maxIntegralWindup, integralWindup.x);
    if (fabs(integralWindup.y) > maxIntegralWindup)
        integralWindup.y = copysign(maxIntegralWindup, integralWindup.y);

    return integralWindup;
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

    return stateEstimate;
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
            x_hat[0] = 0.99999*orientation[2] - 0.99999*stateEstimateCopy.q1 + 3.0*Ts*controlSignal.q1ref - 1.0*stateEstimateCopy.q1*(3.0*Ts - 1.0);
            x_hat[1] = 0.99999*orientation[3] - 0.99999*stateEstimateCopy.q2 + 3.0*Ts*controlSignal.q2ref - 1.0*stateEstimateCopy.q2*(3.0*Ts - 1.0);
            x_hat[2] = 0.27854194999999998*measurement.x + 0.00000060999999999999998*orientation[3] - 0.00000060999999999999998*stateEstimateCopy.q2 + 0.72145805000000002*stateEstimateCopy.x + Ts*stateEstimateCopy.vx;
            x_hat[3] = 0.27854194999999998*measurement.y - 0.00000060999999999999998*orientation[2] + 0.00000060999999999999998*stateEstimateCopy.q1 + 0.72145805000000002*stateEstimateCopy.y + Ts*stateEstimateCopy.vy;
            x_hat[4] = 0.026860019999999998*measurement.x + 0.00001364*orientation[3] - 0.00001364*stateEstimateCopy.q2 - 0.026860019999999998*stateEstimateCopy.x + stateEstimateCopy.vx + 19.62*Ts*stateEstimateCopy.q2;
            x_hat[5] = 0.026860019999999998*measurement.y - 0.00001364*orientation[2] + 0.00001364*stateEstimateCopy.q1 - 0.026860019999999998*stateEstimateCopy.y + stateEstimateCopy.vy - 19.62*Ts*stateEstimateCopy.q1;
            break;
        case 2:
            x_hat[0] = 0.99999*orientation[2] - 0.99999*stateEstimateCopy.q1 + 3.0*Ts*controlSignal.q1ref - 1.0*stateEstimateCopy.q1*(3.0*Ts - 1.0);
            x_hat[1] = 0.99999*orientation[3] - 0.99999*stateEstimateCopy.q2 + 3.0*Ts*controlSignal.q2ref - 1.0*stateEstimateCopy.q2*(3.0*Ts - 1.0);
            x_hat[2] = 0.27854194999999998*measurement.x + 0.00000060999999999999998*orientation[3] - 0.00000060999999999999998*stateEstimateCopy.q2 + 0.72145805000000002*stateEstimateCopy.x + Ts*stateEstimateCopy.vx;
            x_hat[3] = 0.27854194999999998*measurement.y - 0.00000060999999999999998*orientation[2] + 0.00000060999999999999998*stateEstimateCopy.q1 + 0.72145805000000002*stateEstimateCopy.y + Ts*stateEstimateCopy.vy;
            x_hat[4] = 0.026860019999999998*measurement.x + 0.00001364*orientation[3] - 0.00001364*stateEstimateCopy.q2 - 0.026860019999999998*stateEstimateCopy.x + stateEstimateCopy.vx + 19.62*Ts*stateEstimateCopy.q2;
            x_hat[5] = 0.026860019999999998*measurement.y - 0.00001364*orientation[2] + 0.00001364*stateEstimateCopy.q1 - 0.026860019999999998*stateEstimateCopy.y + stateEstimateCopy.vy - 19.62*Ts*stateEstimateCopy.q1;
            break;
        case 3:
            x_hat[0] = 0.99999*orientation[2] - 0.99999*stateEstimateCopy.q1 + 3.0*Ts*controlSignal.q1ref - 1.0*stateEstimateCopy.q1*(3.0*Ts - 1.0);
            x_hat[1] = 0.99999*orientation[3] - 0.99999*stateEstimateCopy.q2 + 3.0*Ts*controlSignal.q2ref - 1.0*stateEstimateCopy.q2*(3.0*Ts - 1.0);
            x_hat[2] = 0.27854194999999998*measurement.x + 0.00000060999999999999998*orientation[3] - 0.00000060999999999999998*stateEstimateCopy.q2 + 0.72145805000000002*stateEstimateCopy.x + Ts*stateEstimateCopy.vx;
            x_hat[3] = 0.27854194999999998*measurement.y - 0.00000060999999999999998*orientation[2] + 0.00000060999999999999998*stateEstimateCopy.q1 + 0.72145805000000002*stateEstimateCopy.y + Ts*stateEstimateCopy.vy;
            x_hat[4] = 0.026860019999999998*measurement.x + 0.00001364*orientation[3] - 0.00001364*stateEstimateCopy.q2 - 0.026860019999999998*stateEstimateCopy.x + stateEstimateCopy.vx + 19.62*Ts*stateEstimateCopy.q2;
            x_hat[5] = 0.026860019999999998*measurement.y - 0.00001364*orientation[2] + 0.00001364*stateEstimateCopy.q1 - 0.026860019999999998*stateEstimateCopy.y + stateEstimateCopy.vy - 19.62*Ts*stateEstimateCopy.q1;
            break;
        case 4:
            x_hat[0] = 0.99999*orientation[2] - 0.99999*stateEstimateCopy.q1 + 3.0*Ts*controlSignal.q1ref - 1.0*stateEstimateCopy.q1*(3.0*Ts - 1.0);
            x_hat[1] = 0.99999*orientation[3] - 0.99999*stateEstimateCopy.q2 + 3.0*Ts*controlSignal.q2ref - 1.0*stateEstimateCopy.q2*(3.0*Ts - 1.0);
            x_hat[2] = 0.27854194999999998*measurement.x + 0.00000060999999999999998*orientation[3] - 0.00000060999999999999998*stateEstimateCopy.q2 + 0.72145805000000002*stateEstimateCopy.x + Ts*stateEstimateCopy.vx;
            x_hat[3] = 0.27854194999999998*measurement.y - 0.00000060999999999999998*orientation[2] + 0.00000060999999999999998*stateEstimateCopy.q1 + 0.72145805000000002*stateEstimateCopy.y + Ts*stateEstimateCopy.vy;
            x_hat[4] = 0.026860019999999998*measurement.x + 0.00001364*orientation[3] - 0.00001364*stateEstimateCopy.q2 - 0.026860019999999998*stateEstimateCopy.x + stateEstimateCopy.vx + 19.62*Ts*stateEstimateCopy.q2;
            x_hat[5] = 0.026860019999999998*measurement.y - 0.00001364*orientation[2] + 0.00001364*stateEstimateCopy.q1 - 0.026860019999999998*stateEstimateCopy.y + stateEstimateCopy.vy - 19.62*Ts*stateEstimateCopy.q1;
            break;
    }


}
*/