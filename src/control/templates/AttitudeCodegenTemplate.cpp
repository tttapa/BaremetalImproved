#include <Attitude.hpp>
#include <math.h> /* sqrt */

#define SQ(value) ((value) * (value))
#define QUAT_0(value) (value.q.w = sqrt(1.0 - SQ(value.q.x) - SQ(value.q.y) - SQ(value.q.z)))

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
            controlSignal.u.x = $c1$u0;
            controlSignal.u.y = $c1$u1;
            controlSignal.u.z = $c1$u2;
            break;
        case 2:
            controlSignal.u.x = $c2$u0;
            controlSignal.u.y = $c2$u1;
            controlSignal.u.z = $c2$u2;
            break;
        case 3:
            controlSignal.u.x = $c3$u0;
            controlSignal.u.y = $c3$u1;
            controlSignal.u.z = $c3$u2;
            break;
        case 4:
            controlSignal.u.x = $c4$u0;
            controlSignal.u.y = $c4$u1;
            controlSignal.u.z = $c4$u2;
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
    integralWindup.q.x += $int0;
    integralWindup.q.y += $int1;
    integralWindup.q.z += $int2;
    if (fabs(integralWindup.q.x) > maxIntegralWindup)
        integralWindup.q.x =
            copysign(maxIntegralWindup, integralWindup.q.x);
    if (fabs(integralWindup.q.y) > maxIntegralWindup)
        integralWindup.q.y =
            copysign(maxIntegralWindup, integralWindup.q.y);
    if (fabs(integralWindup.q.z) > maxIntegralWindup)
        integralWindup.q.z =
            copysign(maxIntegralWindup, integralWindup.q.z);

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
    AttitudeState prediction; /* = Ax + Bu */
    AttitudeState innovation; /* = L (y - Cx) */
    switch (droneConfiguration) {
        case 1:
            prediction.q.x = $c1$p1;
            prediction.q.y = $c1$p2;
            prediction.q.z = $c1$p3;
            prediction.w.x = $c1$p4;
            prediction.w.y = $c1$p5;
            prediction.w.z = $c1$p6;
            prediction.n.x = $c1$p7;
            prediction.n.y = $c1$p8;
            prediction.n.z = $c1$p9;
            innovation.q.x = $c1$i1;
            innovation.q.y = $c1$i2;
            innovation.q.z = $c1$i3;
            innovation.w.x = $c1$i4;
            innovation.w.y = $c1$i5;
            innovation.w.z = $c1$i6;
            innovation.n.x = $c1$i7;
            innovation.n.y = $c1$i8;
            innovation.n.z = $c1$i9;
            break;
        case 2:
            prediction.q.x = $c2$p1;
            prediction.q.y = $c2$p2;
            prediction.q.z = $c2$p3;
            prediction.w.x = $c2$p4;
            prediction.w.y = $c2$p5;
            prediction.w.z = $c2$p6;
            prediction.n.x = $c2$p7;
            prediction.n.y = $c2$p8;
            prediction.n.z = $c2$p9;
            innovation.q.x = $c2$i1;
            innovation.q.y = $c2$i2;
            innovation.q.z = $c2$i3;
            innovation.w.x = $c2$i4;
            innovation.w.y = $c2$i5;
            innovation.w.z = $c2$i6;
            innovation.n.x = $c2$i7;
            innovation.n.y = $c2$i8;
            innovation.n.z = $c2$i9;
            break;
        case 3:
            prediction.q.x = $c3$p1;
            prediction.q.y = $c3$p2;
            prediction.q.z = $c3$p3;
            prediction.w.x = $c3$p4;
            prediction.w.y = $c3$p5;
            prediction.w.z = $c3$p6;
            prediction.n.x = $c3$p7;
            prediction.n.y = $c3$p8;
            prediction.n.z = $c3$p9;
            innovation.q.x = $c3$i1;
            innovation.q.y = $c3$i2;
            innovation.q.z = $c3$i3;
            innovation.w.x = $c3$i4;
            innovation.w.y = $c3$i5;
            innovation.w.z = $c3$i6;
            innovation.n.x = $c3$i7;
            innovation.n.y = $c3$i8;
            innovation.n.z = $c3$i9;
            break;
        case 4:
            prediction.q.x = $c4$p1;
            prediction.q.y = $c4$p2;
            prediction.q.z = $c4$p3;
            prediction.w.x = $c4$p4;
            prediction.w.y = $c4$p5;
            prediction.w.z = $c4$p6;
            prediction.n.x = $c4$p7;
            prediction.n.y = $c4$p8;
            prediction.n.z = $c4$p9;
            innovation.q.x = $c4$i1;
            innovation.q.y = $c4$i2;
            innovation.q.z = $c4$i3;
            innovation.w.x = $c4$i4;
            innovation.w.y = $c4$i5;
            innovation.w.z = $c4$i6;
            innovation.n.x = $c4$i7;
            innovation.n.y = $c4$i8;
            innovation.n.z = $c4$i9;
            break;
        default:
            prediction = {};
            innovation = {};
            break;
    }
    QUAT_0(prediction);
    QUAT_0(innovation);
    stateEstimate.q.w = $x0;
    stateEstimate.q.x = $x1;
    stateEstimate.q.y = $x2;
    stateEstimate.q.z = $x3;
    stateEstimate.w.x = $x4;
    stateEstimate.w.y = $x5;
    stateEstimate.w.z = $x6;
    stateEstimate.n.x = $x7;
    stateEstimate.n.y = $x8;
    stateEstimate.n.z = $x9;
    return stateEstimate;
}
