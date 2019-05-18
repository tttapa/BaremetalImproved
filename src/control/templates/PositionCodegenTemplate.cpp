#include <Position.hpp>
#include <math.h>   /* fabs, copysign */
#include <string.h> /* memcpy */
#include <MathFunctions.hpp>

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
            controlSignal.pitchRollRef.x = $c1$u0;
            controlSignal.pitchRollRef.y = $c1$u1;
            break;
        case 2:
            controlSignal.pitchRollRef.x = $c2$u0;
            controlSignal.pitchRollRef.y = $c2$u1;
            break;
        case 3:
            controlSignal.pitchRollRef.x = $c3$u0;
            controlSignal.pitchRollRef.y = $c3$u1;
            break;
        case 4:
            controlSignal.pitchRollRef.x = $c4$u0;
            controlSignal.pitchRollRef.y = $c4$u1;
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
            controlSignal.pitchRollRef.x = $c1$uBlind0;
            controlSignal.pitchRollRef.y = $c1$uBlind1;
            break;
        case 2:
            controlSignal.pitchRollRef.x = $c2$uBlind0;
            controlSignal.pitchRollRef.y = $c2$uBlind1;
            break;
        case 3:
            controlSignal.pitchRollRef.x = $c3$uBlind0;
            controlSignal.pitchRollRef.y = $c3$uBlind1;
            break;
        case 4:
            controlSignal.pitchRollRef.x = $c4$uBlind0;
            controlSignal.pitchRollRef.y = $c4$uBlind1;
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
    EulerAngles orientation, float timeElapsed, int droneConfiguration) {

    /* Implement jump rejection to preserve a decent drone velocity. */
    HorizontalVelocity v0    = stateEstimate.v;
    HorizontalVelocity v1    = (measurement.p - stateEstimate.p) / timeElapsed;
    HorizontalVelocity absdV = (v1 - v0).abs();
    HorizontalVelocity dAbsV = v1.abs() - v0.abs();

    /* Jump rejection on x-velocity. */
    if ((dAbsV.x <= 0 && absdV.x < V_THRESHOLD_TOWARDS) ||  //
        (dAbsV.x >= 0 && absdV.x < V_THRESHOLD_AWAY))
        stateEstimate.v.x = v1.x;

    /* Jump rejection on y-velocity. */
    if ((dAbsV.y <= 0 && absdV.y < V_THRESHOLD_TOWARDS) ||  //
        (dAbsV.y >= 0 && absdV.y < V_THRESHOLD_AWAY))
        stateEstimate.v.y = v1.y;

    /* Set orientation and position. */
    stateEstimate.q.x = orientation.pitch;
    stateEstimate.q.y = orientation.roll;
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
PositionState PositionController::codegenCurrentStateEstimateBlind(
    PositionState stateEstimate, PositionControlSignal controlSignal,
    EulerAngles orientation) {

    float px = $xBlind0;
    float py = $xBlind1;
    float vx = $xBlind2;
    float vy = $xBlind3;

    stateEstimate.q.x = orientation.pitch;
    stateEstimate.q.y = orientation.roll;
    stateEstimate.p   = Position{px, py};
    stateEstimate.v   = HorizontalVelocity{vx, vy};
    return stateEstimate;
}
