#include <Attitude.hpp>
#include <math.h> /* sqrt */

#define SQ(x) ((x) * (x))
#define QUAT_0(x) (x.q[0] = sqrt(1.0 - SQ(x.q[1]) - SQ(x.q[2]) - SQ(x.q[3])))

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
    AttitudeState prediction; /* = Ax + Bu */
    AttitudeState innovation; /* = L (y - Cx) */
    switch (droneConfiguration) {
        case 1:
            prediction.q[1] = $c1$p1;
            prediction.q[2] = $c1$p2;
            prediction.q[3] = $c1$p3;
            prediction.wx   = $c1$p4;
            prediction.wy   = $c1$p5;
            prediction.wz   = $c1$p6;
            prediction.nx   = $c1$p7;
            prediction.ny   = $c1$p8;
            prediction.nz   = $c1$p9;
            innovation.q[1] = $c1$i1;
            innovation.q[2] = $c1$i2;
            innovation.q[3] = $c1$i3;
            innovation.wx   = $c1$i4;
            innovation.wy   = $c1$i5;
            innovation.wz   = $c1$i6;
            innovation.nx   = $c1$i7;
            innovation.ny   = $c1$i8;
            innovation.nz   = $c1$i9;
            break;
        case 2:
            prediction.q[1] = $c2$p1;
            prediction.q[2] = $c2$p2;
            prediction.q[3] = $c2$p3;
            prediction.wx   = $c2$p4;
            prediction.wy   = $c2$p5;
            prediction.wz   = $c2$p6;
            prediction.nx   = $c2$p7;
            prediction.ny   = $c2$p8;
            prediction.nz   = $c2$p9;
            innovation.q[1] = $c2$i1;
            innovation.q[2] = $c2$i2;
            innovation.q[3] = $c2$i3;
            innovation.wx   = $c2$i4;
            innovation.wy   = $c2$i5;
            innovation.wz   = $c2$i6;
            innovation.nx   = $c2$i7;
            innovation.ny   = $c2$i8;
            innovation.nz   = $c2$i9;
            break;
        case 3:
            prediction.q[1] = $c3$p1;
            prediction.q[2] = $c3$p2;
            prediction.q[3] = $c3$p3;
            prediction.wx   = $c3$p4;
            prediction.wy   = $c3$p5;
            prediction.wz   = $c3$p6;
            prediction.nx   = $c3$p7;
            prediction.ny   = $c3$p8;
            prediction.nz   = $c3$p9;
            innovation.q[1] = $c3$i1;
            innovation.q[2] = $c3$i2;
            innovation.q[3] = $c3$i3;
            innovation.wx   = $c3$i4;
            innovation.wy   = $c3$i5;
            innovation.wz   = $c3$i6;
            innovation.nx   = $c3$i7;
            innovation.ny   = $c3$i8;
            innovation.nz   = $c3$i9;
            break;
        case 4:
            prediction.q[1] = $c4$p1;
            prediction.q[2] = $c4$p2;
            prediction.q[3] = $c4$p3;
            prediction.wx   = $c4$p4;
            prediction.wy   = $c4$p5;
            prediction.wz   = $c4$p6;
            prediction.nx   = $c4$p7;
            prediction.ny   = $c4$p8;
            prediction.nz   = $c4$p9;
            innovation.q[1] = $c4$i1;
            innovation.q[2] = $c4$i2;
            innovation.q[3] = $c4$i3;
            innovation.wx   = $c4$i4;
            innovation.wy   = $c4$i5;
            innovation.wz   = $c4$i6;
            innovation.nx   = $c4$i7;
            innovation.ny   = $c4$i8;
            innovation.nz   = $c4$i9;
            break;
    }
    QUAT_0(prediction);
    QUAT_0(innovation);
    stateEstimate.q[0] = $x0;
    stateEstimate.q[1] = $x1;
    stateEstimate.q[2] = $x2;
    stateEstimate.q[3] = $x3;
    stateEstimate.wx   = $x4;
    stateEstimate.wy   = $x5;
    stateEstimate.wz   = $x6;
    stateEstimate.nx   = $x7;
    stateEstimate.ny   = $x8;
    stateEstimate.nz   = $x9;
    return stateEstimate;
}
