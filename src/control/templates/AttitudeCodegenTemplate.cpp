#include <Attitude.hpp>

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AttitudeControlSignal
AttitudeController::codegenControlSignal(AttitudeState stateEstimate,
                                         AttitudeReference reference,
                                         AttitudeIntegralWindup integralWindup,
                                         int droneConfiguration) {

    /* Calculate control signal based on drone configuration. */
    AttitudeControlSignal controlSignal;
    switch (droneConfiguration) {
        case 1:
            controlSignal.ux = $c1$u0;
            controlSignal.uy = $c1$u1;
            controlSignal.uz = $c1$u2;
            break;
        case 2:
            controlSignal.ux = $c2$u0;
            controlSignal.uy = $c2$u1;
            controlSignal.uz = $c2$u2;
            break;
        case 3:
            controlSignal.ux = $c3$u0;
            controlSignal.uy = $c3$u1;
            controlSignal.uz = $c3$u2;
            break;
        case 4:
            controlSignal.ux = $c4$u0;
            controlSignal.uy = $c4$u1;
            controlSignal.uz = $c4$u2;
            break;
        default: controlSignal = {};
    }
    return controlSignal;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AttitudeIntegralWindup
AttitudeController::codegenIntegralWindup(AttitudeIntegralWindup integralWindup,
                                          AttitudeReference reference,
                                          AttitudeState stateEstimate,
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
    integralWindup.q1 += $int0;
    integralWindup.q2 += $int1;
    integralWindup.q3 += $int2;
    if (fabs(integralWindup.q1) > maxIntegralWindup)
        integralWindup.q1 = copysign(maxIntegralWindup, integralWindup.q1);
    if (fabs(integralWindup.q2) > maxIntegralWindup)
        integralWindup.q2 = copysign(maxIntegralWindup, integralWindup.q2);
    if (fabs(integralWindup.q3) > maxIntegralWindup)
        integralWindup.q3 = copysign(maxIntegralWindup, integralWindup.q3);

    return integralWindup;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AttitudeState
AttitudeController::codegenNextStateEstimate(AttitudeState stateEstimate,
                                             AttitudeControlSignal controlSignal,
                                             AttitudeMeasurement measurement,
                                             int droneConfiguration) {

    /* Calculate next state using Kalman Filter based on drone configuration. */
    switch (droneConfiguration) {
        case 1:
            stateEstimate.yaw    = $c1$x0;
            stateEstimate.pitch  = $c1$x1;
            stateEstimate.roll   = $c1$x2;
            stateEstimate.wyaw   = $c1$x3;
            stateEstimate.wpitch = $c1$x4;
            stateEstimate.wroll  = $c1$x5;
            stateEstimate.nyaw   = $c1$x6;
            stateEstimate.npitch = $c1$x7;
            stateEstimate.nroll  = $c1$x8;
            break;
        case 2:
            stateEstimate.yaw    = $c2$x0;
            stateEstimate.pitch  = $c2$x1;
            stateEstimate.roll   = $c2$x2;
            stateEstimate.wyaw   = $c2$x3;
            stateEstimate.wpitch = $c2$x4;
            stateEstimate.wroll  = $c2$x5;
            stateEstimate.nyaw   = $c2$x6;
            stateEstimate.npitch = $c2$x7;
            stateEstimate.nroll  = $c2$x8;
            break;
        case 3:
            stateEstimate.yaw    = $c3$x0;
            stateEstimate.pitch  = $c3$x1;
            stateEstimate.roll   = $c3$x2;
            stateEstimate.wyaw   = $c3$x3;
            stateEstimate.wpitch = $c3$x4;
            stateEstimate.wroll  = $c3$x5;
            stateEstimate.nyaw   = $c3$x6;
            stateEstimate.npitch = $c3$x7;
            stateEstimate.nroll  = $c3$x8;
            break;
        case 4:
            stateEstimate.yaw    = $c4$x0;
            stateEstimate.pitch  = $c4$x1;
            stateEstimate.roll   = $c4$x2;
            stateEstimate.wyaw   = $c4$x3;
            stateEstimate.wpitch = $c4$x4;
            stateEstimate.wroll  = $c4$x5;
            stateEstimate.nyaw   = $c4$x6;
            stateEstimate.npitch = $c4$x7;
            stateEstimate.nroll  = $c4$x8;
            break;
        default:
            stateEstimate = {};
            break;
    }
    return stateEstimate;
}
