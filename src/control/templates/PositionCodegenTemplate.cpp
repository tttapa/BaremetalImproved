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
PositionController::codegenControlSignal(PositionState stateEstimate,
                                         PositionReference reference,
                                         PositionIntegralWindup integralWindup,
                                         int droneConfiguration) {

    /* Calculate controller output based on drone configuration. */
    PositionControlSignal controlSignal;
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
    return controlSignal;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
PositionIntegralWindup
PositionController::codegenIntegralWindup(PositionIntegralWindup integralWindup,
                                          PositionReference reference,
                                          PositionState stateEstimate,
                                          int droneConfiguration) {

    /* Set maximum integral windup based on drone configuration. */
    real_t maxIntegralWindup;
    switch (droneConfiguration) {
        case 1: maxIntegralWindup = $c1$maxWindup; break;
        case 2: maxIntegralWindup = $c2$maxWindup; break;
        case 3: maxIntegralWindup = $c3$maxWindup; break;
        case 4: maxIntegralWindup = $c4$maxWindup; break;
        default: maxIntegralWindup = 0.0;
    }

    /* Update integral windup. */
    integralWindup.x += $int0;
    integralWindup.y += $int1;
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
PositionState
PositionController::codegenCurrentStateEstimate(PositionState stateEstimate,
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
    stateEstimate.q1  = orientation[1];
    stateEstimate.q2  = orientation[2];
    stateEstimate.p.x = measurement.p.x;
    stateEstimate.p.y = measurement.p.y;

    /* Drone configuration unused. */
    (void) droneConfiguration;

    return stateEstimate;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template.
 */
PositionState
PositionController::codegenCurrentStateEstimateBlind(PositionStateBlind stateEstimateBlind,
                                                     PositionControlSignalBlind controlSignalBlind) {

    PositionStateBlind stateEstimateBlindCopy = stateEstimateBlind;

    stateEstimateBlind.p.x = $x0;
    stateEstimateBlind.p.y = $x1;
    stateEstimateBlind.vx  = $x2;
    stateEstimateBlind.vy  = $x3;

    return PositionState{controlSignalBlind.q1, controlSignalBlind.q2,
                         stateEstimateBlind.p, stateEstimateBlind.vx,
                         stateEstimateBlind.vy};
}
