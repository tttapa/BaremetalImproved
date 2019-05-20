#include <MathFunctions.hpp>
#include <Position.hpp>
#include <MiscInstances.hpp>
#include <math.h>   /* fabs, copysign */
#include <string.h> /* memcpy */

/**
 * If the drone's velocity changes by more than 0.30 m/s between measurements,
 * and the velocity is moving away from 0 m/s, then reject the jump.
 */
static constexpr float V_THRESHOLD_AWAY = 0.30;

/**
 * If the drone's velocity changes by more than 0.50 m/s between measurements,
 * and the velocity is moving toward 0 m/s, then reject the jump.
 */
static constexpr float V_THRESHOLD_TOWARDS = 0.50;

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
            controlSignal.q12.x = $c1$u0;
            controlSignal.q12.y = $c1$u1;
            break;
        case 2:
            controlSignal.q12.x = $c2$u0;
            controlSignal.q12.y = $c2$u1;
            break;
        case 3:
            controlSignal.q12.x = $c3$u0;
            controlSignal.q12.y = $c3$u1;
            break;
        case 4:
            controlSignal.q12.x = $c4$u0;
            controlSignal.q12.y = $c4$u1;
            break;
        default: controlSignal = {};
    }
    (void) integralWindup;
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
            controlSignal.q12.x = $c1$uBlind0;
            controlSignal.q12.y = $c1$uBlind1;
            break;
        case 2:
            controlSignal.q12.x = $c2$uBlind0;
            controlSignal.q12.y = $c2$uBlind1;
            break;
        case 3:
            controlSignal.q12.x = $c3$uBlind0;
            controlSignal.q12.y = $c3$uBlind1;
            break;
        case 4:
            controlSignal.q12.x = $c4$uBlind0;
            controlSignal.q12.y = $c4$uBlind1;
            break;
        default: controlSignal = {};
    }
    (void) integralWindup;
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
    float maxIntegralWindup;
    switch (droneConfiguration) {
        case 1: maxIntegralWindup = $c1$maxWindup; break;
        case 2: maxIntegralWindup = $c2$maxWindup; break;
        case 3: maxIntegralWindup = $c3$maxWindup; break;
        case 4: maxIntegralWindup = $c4$maxWindup; break;
        default: maxIntegralWindup = 0.0;
    }

    /* Update integral windup. */
    integralWindup.p.x += $int0;
    integralWindup.p.y += $int1;
    if (std2::absf(integralWindup.p.x) > maxIntegralWindup)
        integralWindup.p.x = copysign(maxIntegralWindup, integralWindup.p.x);
    if (std2::absf(integralWindup.p.y) > maxIntegralWindup)
        integralWindup.p.y = copysign(maxIntegralWindup, integralWindup.p.y);

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
    float maxIntegralWindup;
    switch (droneConfiguration) {
        case 1: maxIntegralWindup = $c1$maxWindup; break;
        case 2: maxIntegralWindup = $c2$maxWindup; break;
        case 3: maxIntegralWindup = $c3$maxWindup; break;
        case 4: maxIntegralWindup = $c4$maxWindup; break;
        default: maxIntegralWindup = 0.0;
    }

    /* Update integral windup. */
    integralWindup.p.x += $intBlind0;
    integralWindup.p.y += $intBlind1;
    if (std2::absf(integralWindup.p.x) > maxIntegralWindup)
        integralWindup.p.x = copysign(maxIntegralWindup, integralWindup.p.x);
    if (std2::absf(integralWindup.p.y) > maxIntegralWindup)
        integralWindup.p.y = copysign(maxIntegralWindup, integralWindup.p.y);

    return integralWindup;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template.
 */
PositionState PositionController::codegenCurrentStateEstimate(
    PositionState stateEstimate, PositionMeasurement measurement,
    Quaternion orientation, float timeElapsed, int droneConfiguration) {

    /* Bad measurement rejection. */
    bool ignoreMeasurement = false;
    float threshold = 0.07;
    if(std2::absf(measurement.p.x - stateEstimate.p.x) > threshold ||
       std2::absf(measurement.p.y - stateEstimate.p.y) > threshold) {
           ignoreMeasurement = true;
    }

    /* Set orientation. */
    stateEstimate.q.x = orientation.x - 0.5 * biasManager.getRollBias();
    stateEstimate.q.y = orientation.y - 0.5 * biasManager.getPitchBias();

    /* Set position / velocity if measurement is good. */
    if(!ignoreMeasurement) {
        stateEstimate.p = measurement.p;
        stateEstimate.v = (measurement.p - stateEstimate.p) / timeElapsed;
    }

    // /* Implement jump rejection to preserve a decent drone velocity. */
    // HorizontalVelocity v0 = stateEstimate.v;
    // HorizontalVelocity v1;

    // /* Set orientation and position (ignore if position jumps). */
    // stateEstimate.q.x = orientation.x - 0.5 * biasManager.getRollBias();
    // stateEstimate.q.y = orientation.y - 0.5 * biasManager.getPitchBias();

    // if(!xIgnoreVelocity)
    //     stateEstimate.p.x = measurement.p.x;
    // if(!yIgnoreVelocity)
    //     stateEstimate.p.y = measurement.p.y;


    // if (timeElapsed > 0)
    //     v1 = (measurement.p - stateEstimate.p) / timeElapsed;
    // else
    //     v1 = v0;

    // HorizontalVelocity absdV = (v1 - v0).abs();
    // HorizontalVelocity dAbsV = v1.abs() - v0.abs();

    // /* Jump rejection on x-velocity. */
    // bool xIgnoreVelocity = true;
    // if ((dAbsV.x <= 0 && absdV.x < V_THRESHOLD_TOWARDS) ||  //
    //     (dAbsV.x >= 0 && absdV.x < V_THRESHOLD_AWAY)) {
    //     stateEstimate.v.x = v1.x;
    //     xIgnoreVelocity = false;
    // }

    // /* Jump rejection on y-velocity. */
    // bool yIgnoreVelocity = true;
    // if ((dAbsV.y <= 0 && absdV.y < V_THRESHOLD_TOWARDS) ||  //
    //     (dAbsV.y >= 0 && absdV.y < V_THRESHOLD_AWAY)) {
    //     stateEstimate.v.y = v1.y;
    //     yIgnoreVelocity = false;
    // }

    // /* Set orientation and position (ignore if position jumps). */
    // stateEstimate.q.x = orientation.x - 0.5 * biasManager.getRollBias();
    // stateEstimate.q.y = orientation.y - 0.5 * biasManager.getPitchBias();
    // if(!xIgnoreVelocity)
    //     stateEstimate.p.x = measurement.p.x;
    // if(!yIgnoreVelocity)
    //     stateEstimate.p.y = measurement.p.y;


    /* Drone configuration unused. */
    (void) droneConfiguration;

    return stateEstimate;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template.
 */
PositionState PositionController::codegenCurrentStateEstimateBlind(
    PositionState stateEstimate, PositionControlSignal controlSignal, Quaternion orientation) {

    stateEstimate.q.x = orientation.x;// - 0.5 * getRollBias();
    stateEstimate.q.y = orientation.y;// - 0.5 * getPitchBias();
    stateEstimate.p.x = $xBlind0;
    stateEstimate.p.y = $xBlind1;
    stateEstimate.v.x = $xBlind2;
    stateEstimate.v.y = $xBlind3;
    return stateEstimate;
}
