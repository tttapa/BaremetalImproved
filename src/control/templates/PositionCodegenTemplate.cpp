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
PositionControlSignal PositionController::codegenControlSignal(
    PositionState stateEstimate, PositionReference reference,
    PositionIntegralWindup integralWindup, int droneConfiguration) {

    /* Calculate controller output based on drone configuration. */
    PositionControlSignal controlSignal;
    switch (droneConfiguration) {
        case 1:
            controlSignal.q12[0] = $c1$u0;
            controlSignal.q12[1] = $c1$u1;
            break;
        case 2:
            controlSignal.q12[0] = $c2$u0;
            controlSignal.q12[1] = $c2$u1;
            break;
        case 3:
            controlSignal.q12[0] = $c3$u0;
            controlSignal.q12[1] = $c3$u1;
            break;
        case 4:
            controlSignal.q12[0] = $c4$u0;
            controlSignal.q12[1] = $c4$u1;
            break;
        default: controlSignal = {};
    }
    (void)integralWindup;
    return controlSignal;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
PositionControlSignal PositionController::codegenControlSignalBlind(
    PositionState stateEstimate, PositionReference reference,
    PositionIntegralWindup integralWindup, int droneConfiguration) {

    /* Calculate controller output based on drone configuration. */
    PositionControlSignal controlSignal;
    switch (droneConfiguration) {
        case 1:
            controlSignal.q12[0] = $c1$uBlind0;
            controlSignal.q12[1] = $c1$uBlind1;
            break;
        case 2:
            controlSignal.q12[0] = $c2$uBlind0;
            controlSignal.q12[1] = $c2$uBlind1;
            break;
        case 3:
            controlSignal.q12[0] = $c3$uBlind0;
            controlSignal.q12[1] = $c3$uBlind1;
            break;
        case 4:
            controlSignal.q12[0] = $c4$uBlind0;
            controlSignal.q12[1] = $c4$uBlind1;
            break;
        default: controlSignal = {};
    }
    (void)integralWindup;
    return controlSignal;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
PositionIntegralWindup PositionController::codegenIntegralWindup(
    PositionIntegralWindup integralWindup, PositionReference reference,
    PositionState stateEstimate, int droneConfiguration) {

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
    integralWindup.p[0] += $int0;
    integralWindup.p[1] += $int1;
    if (fabs(integralWindup.p[0]) > maxIntegralWindup)
        integralWindup.p[0] = copysign(maxIntegralWindup, integralWindup.p[0]);
    if (fabs(integralWindup.p[1]) > maxIntegralWindup)
        integralWindup.p[1] = copysign(maxIntegralWindup, integralWindup.p[1]);

    return integralWindup;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
PositionIntegralWindup PositionController::codegenIntegralWindupBlind(
    PositionIntegralWindup integralWindup, PositionReference reference,
    PositionState stateEstimate, int droneConfiguration) {

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
    integralWindup.p[0] += $intBlind0;
    integralWindup.p[1] += $intBlind1;
    if (fabs(integralWindup.p[0]) > maxIntegralWindup)
        integralWindup.p[0] = copysign(maxIntegralWindup, integralWindup.p[0]);
    if (fabs(integralWindup.p[1]) > maxIntegralWindup)
        integralWindup.p[1] = copysign(maxIntegralWindup, integralWindup.p[1]);

    return integralWindup;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template.
 */
PositionState PositionController::codegenCurrentStateEstimate(
    PositionState stateEstimate, PositionMeasurement measurement,
    Quaternion orientation, real_t timeElapsed, int droneConfiguration) {

    /* Implement jump rejection to preserve a decent drone velocity. */
    HorizontalVelocity v0    = stateEstimate.v;
    HorizontalVelocity v1    = (measurement.p - stateEstimate.p) / timeElapsed;
    HorizontalVelocity absdV = abs(v1 - v0);
    HorizontalVelocity dAbsV = abs(v1) - abs(v0);

    /* Jump rejection on x-velocity. */
    if ((dAbsV[0] <= 0 && absdV[0] < V_THRESHOLD_TOWARDS) ||  //
        (dAbsV[0] >= 0 && absdV[0] < V_THRESHOLD_AWAY))
        stateEstimate.v[0] = v1[0];

    /* Jump rejection on y-velocity. */
    if ((dAbsV[1] <= 0 && absdV[1] < V_THRESHOLD_TOWARDS) ||  //
        (dAbsV[1] >= 0 && absdV[1] < V_THRESHOLD_AWAY))
        stateEstimate.v[1] = v1[1];

    /* Set orientation and position. */
    stateEstimate.q[0] = orientation[1];
    stateEstimate.q[1] = orientation[2];
    stateEstimate.p[0] = measurement.p[0];
    stateEstimate.p[1] = measurement.p[1];

    /* Drone configuration unused. */
    (void) droneConfiguration;

    return stateEstimate;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template.
 */
PositionState PositionController::codegenCurrentStateEstimateBlind(
    PositionStateBlind stateEstimateBlind,
    PositionControlSignalBlind controlSignalBlind, Quaternion orientation) {

    PositionStateBlind stateEstimateBlindCopy = stateEstimateBlind;

    stateEstimateBlind.p[0] = $xBlind0;
    stateEstimateBlind.p[1] = $xBlind1;
    stateEstimateBlind.v[0] = $xBlind2;
    stateEstimateBlind.v[1] = $xBlind3;

    PositionState stateEstimate = {};
    stateEstimate.q[0]          = orientation[1];
    stateEstimate.q[1]          = orientation[2];
    stateEstimate.p             = stateEstimateBlind.p;
    stateEstimate.v             = stateEstimateBlind.v;
    return stateEstimate;
}
