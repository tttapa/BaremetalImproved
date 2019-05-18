#include <Attitude.hpp>
#include <MathFunctions.hpp>

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AttitudeControlSignal AttitudeController::codegenControlSignal(
    AttitudeState stateEstimate, AttitudeReference reference,
    AttitudeIntegralWindup integralWindup, int droneConfiguration) {

    /* Calculate control signal based on drone configuration. */
    AttitudeControlSignal controlSignal;
    switch (droneConfiguration) {
        case 1:
            controlSignal.uEul.yaw   = $c1$u0;
            controlSignal.uEul.pitch = $c1$u1;
            controlSignal.uEul.roll  = $c1$u2;
            break;
        case 2:
            controlSignal.uEul.yaw   = $c2$u0;
            controlSignal.uEul.pitch = $c2$u1;
            controlSignal.uEul.roll  = $c2$u2;
            break;
        case 3:
            controlSignal.uEul.yaw   = $c3$u0;
            controlSignal.uEul.pitch = $c3$u1;
            controlSignal.uEul.roll  = $c3$u2;
            break;
        case 4:
            controlSignal.uEul.yaw   = $c4$u0;
            controlSignal.uEul.pitch = $c4$u1;
            controlSignal.uEul.roll  = $c4$u2;
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
AttitudeIntegralWindup AttitudeController::codegenIntegralWindup(
    AttitudeIntegralWindup integralWindup, AttitudeReference reference,
    AttitudeState stateEstimate, int droneConfiguration) {

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
    integralWindup.eul.yaw += $int0;
    integralWindup.eul.pitch += $int1;
    integralWindup.eul.roll += $int2;
    if (std2::absf(integralWindup.eul.yaw) > maxIntegralWindup)
        integralWindup.eul.yaw =
            copysign(maxIntegralWindup, integralWindup.eul.yaw);
    if (std2::absf(integralWindup.eul.pitch) > maxIntegralWindup)
        integralWindup.eul.pitch =
            copysign(maxIntegralWindup, integralWindup.eul.pitch);
    if (std2::absf(integralWindup.eul.roll) > maxIntegralWindup)
        integralWindup.eul.roll =
            copysign(maxIntegralWindup, integralWindup.eul.roll);

    return integralWindup;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AttitudeState AttitudeController::codegenNextStateEstimate(
    AttitudeState stateEstimate, AttitudeControlSignal controlSignal,
    AttitudeMeasurement measurement, int droneConfiguration) {

    /* Calculate next state using Kalman Filter based on drone configuration. */
    switch (droneConfiguration) {
        case 1:
            stateEstimate.eul.yaw    = $c1$x0;
            stateEstimate.eul.pitch  = $c1$x1;
            stateEstimate.eul.roll   = $c1$x2;
            stateEstimate.wEul.yaw   = $c1$x3;
            stateEstimate.wEul.pitch = $c1$x4;
            stateEstimate.wEul.roll  = $c1$x5;
            stateEstimate.nEul.yaw   = $c1$x6;
            stateEstimate.nEul.pitch = $c1$x7;
            stateEstimate.nEul.roll  = $c1$x8;
            break;
        case 2:
            stateEstimate.eul.yaw    = $c2$x0;
            stateEstimate.eul.pitch  = $c2$x1;
            stateEstimate.eul.roll   = $c2$x2;
            stateEstimate.wEul.yaw   = $c2$x3;
            stateEstimate.wEul.pitch = $c2$x4;
            stateEstimate.wEul.roll  = $c2$x5;
            stateEstimate.nEul.yaw   = $c2$x6;
            stateEstimate.nEul.pitch = $c2$x7;
            stateEstimate.nEul.roll  = $c2$x8;
            break;
        case 3:
            stateEstimate.eul.yaw    = $c3$x0;
            stateEstimate.eul.pitch  = $c3$x1;
            stateEstimate.eul.roll   = $c3$x2;
            stateEstimate.wEul.yaw   = $c3$x3;
            stateEstimate.wEul.pitch = $c3$x4;
            stateEstimate.wEul.roll  = $c3$x5;
            stateEstimate.nEul.yaw   = $c3$x6;
            stateEstimate.nEul.pitch = $c3$x7;
            stateEstimate.nEul.roll  = $c3$x8;
            break;
        case 4:
            stateEstimate.eul.yaw    = $c4$x0;
            stateEstimate.eul.pitch  = $c4$x1;
            stateEstimate.eul.roll   = $c4$x2;
            stateEstimate.wEul.yaw   = $c4$x3;
            stateEstimate.wEul.pitch = $c4$x4;
            stateEstimate.wEul.roll  = $c4$x5;
            stateEstimate.nEul.yaw   = $c4$x6;
            stateEstimate.nEul.pitch = $c4$x7;
            stateEstimate.nEul.roll  = $c4$x8;
            break;
        default:
            stateEstimate = {};
            break;
    }
    return stateEstimate;
}
