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

/**
 * If the drone's velocity changes by more than 0.30 m/s between measurements,
 * and the velocity is moving away from 0 m/s, then reject the jump.
 */
static constexpr real_t V_THRESHOLD_AWAY = 0.30;

/**
 * If the drone's velocity changes by more than 0.50 m/s between measurements,
 * and the velocity is moving toward 0 m/s, then reject the jump.
 */
static constexpr real_t V_THRESHOLD_TOWARDS = 0.50;

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
PositionControlSignal
codegenControlSignal(PositionState stateEstimate, PositionReference reference,
                     PositionIntegralWindup integralWindup,
                     int droneConfiguration) {

    /* Calculate controller output based on drone configuration. */
    PositionControlSignal controlSignal;
    switch (droneConfiguration) {
        case 1:
            controlSignal.q1ref = 0.14859985000000001*stateEstimate.p.y - 0.79794841000000005*stateEstimate.q1 - 0.14859985000000001*reference.p.y + 0.16614145*stateEstimate.vy - 0.001*integralWindup.y;
            controlSignal.q2ref = 0.14859985000000001*reference.p.x - 0.79794841000000005*stateEstimate.q2 - 0.14859985000000001*stateEstimate.p.x - 0.16614145*stateEstimate.vx + 0.001*integralWindup.x;
            break;
        case 2:
            controlSignal.q1ref = 0.14859985000000001*stateEstimate.p.y - 0.79794841000000005*stateEstimate.q1 - 0.14859985000000001*reference.p.y + 0.16614145*stateEstimate.vy - 0.001*integralWindup.y;
            controlSignal.q2ref = 0.14859985000000001*reference.p.x - 0.79794841000000005*stateEstimate.q2 - 0.14859985000000001*stateEstimate.p.x - 0.16614145*stateEstimate.vx + 0.001*integralWindup.x;
            break;
        case 3:
            controlSignal.q1ref = 0.12327784*stateEstimate.p.y - 0.72226261999999997*stateEstimate.q1 - 0.12327784*reference.p.y + 0.14728732*stateEstimate.vy - 0.01*integralWindup.y;
            controlSignal.q2ref = 0.12327784*reference.p.x - 0.72226261999999997*stateEstimate.q2 - 0.12327784*stateEstimate.p.x - 0.14728732*stateEstimate.vx + 0.01*integralWindup.x;
            break;
        case 4:
            controlSignal.q1ref = 0.089303110000000005*stateEstimate.p.y - 0.60622721999999996*stateEstimate.q1 - 0.089303110000000005*reference.p.y + 0.12103102*stateEstimate.vy - 0.01*integralWindup.y;
            controlSignal.q2ref = 0.089303110000000005*reference.p.x - 0.60622721999999996*stateEstimate.q2 - 0.089303110000000005*stateEstimate.p.x - 0.12103102*stateEstimate.vx + 0.01*integralWindup.x;
            break;
        default: controlSignal = {};
    }
    return controlSignal;
}

PositionIntegralWindup
codegenIntegralWindup(PositionIntegralWindup integralWindup,
                      PositionReference reference, PositionState stateEstimate,
                      int droneConfiguration) {

    /* Set maximum integral windup based on drone configuration. */
    real_t maxIntegralWindup;
    switch (droneConfiguration) {
        case 1: maxIntegralWindup = 10; break;
        case 2: maxIntegralWindup = 10; break;
        case 3: maxIntegralWindup = 10; break;
        case 4: maxIntegralWindup = 10; break;
        default: maxIntegralWindup = 0.0;
    }

    /* Update integral windup. */
    integralWindup.x += 0.11764705882352941*reference.p.x - 0.11764705882352941*stateEstimate.p.x;
    integralWindup.y += 0.11764705882352941*reference.p.y - 0.11764705882352941*stateEstimate.p.y;
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
                                          real_t timeElapsed,
                                          int droneConfiguration) {

    /* Implement jump rejection to preserve a decent drone velocity. */
    real_t vx0 = stateEstimate.vx;
    real_t vy0 = stateEstimate.vy;
    real_t vx1 = (measurement.p.x - stateEstimate.p.x) / timeElapsed;
    real_t vy1 = (measurement.p.y - stateEstimate.p.y) / timeElapsed;

    /* Jump rejection on x-velocity. */
    if (fabs(vx1) - fabs(vx0) <= 0 && fabs(vx1 - vx0) < V_THRESHOLD_TOWARDS)
        stateEstimate.vx = vx1;
    else if (fabs(vx1) - fabs(vx0) >= 0 && fabs(vx1 - vx0) < V_THRESHOLD_AWAY)
        stateEstimate.vx = vx1;

    /* Jump rejection on y-velocity. */
    if (fabs(vy1) - fabs(vy0) <= 0 && fabs(vy1 - vy0) < V_THRESHOLD_TOWARDS)
        stateEstimate.vy = vy1;
    else if (fabs(vy1) - fabs(vy0) >= 0 && fabs(vy1 - vy0) < V_THRESHOLD_AWAY)
        stateEstimate.vy = vy1;

    /* Set orientation and position. */
    stateEstimate.q1 = orientation[1];
    stateEstimate.q2 = orientation[2];
    stateEstimate.p.x  = measurement.p.x;
    stateEstimate.p.y  = measurement.p.y;

    /* Drone configuration unused. */
    (void) droneConfiguration;

    return stateEstimate;
}

PositionState codegenCurrentStateEstimateBlind(
    PositionStateBlind stateEstimateBlind,
    PositionControlSignalBlind controlSignalBlind) {

    PositionStateBlind stateEstimateBlindCopy = stateEstimateBlind;

    stateEstimateBlind.p.x  = 0.00017318692182755456*controlSignalBlind.q2 + stateEstimateBlindCopy.p.x + 0.0042016806722689076*stateEstimateBlindCopy.vx;
    stateEstimateBlind.p.y  = stateEstimateBlindCopy.p.y - 0.00017318692182755456*controlSignalBlind.q1 + 0.0042016806722689076*stateEstimateBlindCopy.vy;
    stateEstimateBlind.vx = 0.082436974789915966*controlSignalBlind.q2 + stateEstimateBlindCopy.vx;
    stateEstimateBlind.vy = stateEstimateBlindCopy.vy - 0.082436974789915966*controlSignalBlind.q1;

    return {PositionState{controlSignalBlind.q1, controlSignalBlind.q2,
                          stateEstimateBlind.p,
                          stateEstimateBlind.vx, stateEstimateBlind.vy}};
}

