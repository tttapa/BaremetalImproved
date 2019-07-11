#include <MathFunctions.hpp>
#include <MiscInstances.hpp>
#include <Position.hpp>
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

    //    DEMO CODE!!! Very bad observer for velocity
    //
    //    /* Implement jump rejection to preserve a decent drone velocity. */
    //    HorizontalVelocity v0 = stateEstimate.v;
    //    HorizontalVelocity v1;
    //    if (timeElapsed > 0)
    //        v1 = (measurement.p - stateEstimate.p) / timeElapsed;
    //    else
    //        v1 = v0;
    //
    //    HorizontalVelocity absdV = (v1 - v0).abs();
    //    HorizontalVelocity dAbsV = v1.abs() - v0.abs();
    //
    //    /* Jump rejection on x-velocity. */
    //    if ((dAbsV.x <= 0 && absdV.x < V_THRESHOLD_TOWARDS) ||  //
    //        (dAbsV.x >= 0 && absdV.x < V_THRESHOLD_AWAY))
    //        stateEstimate.v.x = v1.x;
    //
    //    /* Jump rejection on y-velocity. */
    //    if ((dAbsV.y <= 0 && absdV.y < V_THRESHOLD_TOWARDS) ||  //
    //        (dAbsV.y >= 0 && absdV.y < V_THRESHOLD_AWAY))
    //        stateEstimate.v.y = v1.y;
    //

    // Test 1: it loiters within a 3x3 grid, except if it jumps a square...
    //         I'm guessing it's because velocity lags too much, so we're
    //         increasing the alphas
    //
    //         Another problem was that the EMA (posFilt) is not initialized at
    //         start, so the first control action is crazy large. We'll let the
    //         second EMA (stateEstimate.v) be initialized to zero.
    //float alpha1     = 0.2;
    //float alpha2     = 0.2;

    // Test 2: loitering improved with adjusted alphas, we're gonna test
    //         different lambdas and see which one works the best

    /* Filter velocity. */
    float alpha1 = 0.4;
    float alpha2 = 0.6;
    Position posFilt = measurement.p * alpha1 + PositionController::getPosFilt() * (1 - alpha1);
    HorizontalVelocity vFromPosFilt = (posFilt - PositionController::getPosFilt()) / timeElapsed;
    stateEstimate.v = vFromPosFilt * alpha2 + stateEstimate.v * (1 - alpha2);

    /* Set orientation and position. */
    stateEstimate.q.x = orientation.x - 0.5 * biasManager.getRollBias();
    stateEstimate.q.y = orientation.y - 0.5 * biasManager.getPitchBias();
    stateEstimate.p.x = measurement.p.x;
    stateEstimate.p.y = measurement.p.y;

    /* Set last filtered measurement. */
    PositionController::setPosFilt(posFilt);

    /* Drone configuration unused. */
    (void) droneConfiguration;

    return stateEstimate;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template.
 */
PositionState PositionController::codegenCurrentStateEstimateBlind(
    PositionState stateEstimate, PositionControlSignal controlSignal,
    Quaternion orientation) {

    stateEstimate.q.x = orientation.x;  // - 0.5 * getRollBias();
    stateEstimate.q.y = orientation.y;  // - 0.5 * getPitchBias();
    stateEstimate.p.x = $xBlind0;
    stateEstimate.p.y = $xBlind1;
    stateEstimate.v.x = $xBlind2;
    stateEstimate.v.y = $xBlind3;
    return stateEstimate;
}
