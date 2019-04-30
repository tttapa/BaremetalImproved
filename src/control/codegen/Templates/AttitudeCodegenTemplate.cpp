#include <Attitude.hpp>
#include <math.h> /* sqrt */

#define SQ(x) ((x) * (x))
#define CALCULATE_QUAT_0(x)                                                    \
    x.q[0] = sqrt(1.0 - SQ(x.q[1]) - SQ(x.q[2]) - SQ(x.q[3]))

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AttitudeControlSignal AttitudeController::codegenControlSignal(
    AttitudeState stateEstimate, AttitudeReference reference,
    AttitudeIntegralWindup integralWindup, int droneConfiguration) {

    AttitudeControlSignal controlSignal;

    /* Generated calculations for control signal. */
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

/* Don't use integral action if tunerValue < 0.0. */
//if(tunerValue < 0.0)
//	y_int_max = 0.0;
// (void) tunerValue;

AttitudeIntegralWindup AtttitudeController::codegenIntegralWindup(
    AttitudeIntegralWindup integralWindup, AttitudeReference reference,
    AttitudeState stateEstimate, int droneConfiguration) {

    real_t maxIntegralWindup;

    switch (droneConfiguration) {
        case 1: maxIntegralWindup = $c1$maxWindup; break;
        case 2: maxIntegralWindup = $c2$maxWindup; break;
        case 3: maxIntegralWindup = $c3$maxWindup; break;
        case 4: maxIntegralWindup = $c4$maxWindup; break;
        default: maxIntegralWindup = 0.0;
    }

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
AttitudeState AttitudeController::codegenNextStateEstimate(
    AttitudeState stateEstimate, AttitudeControlSignal controlSignal,
    AttitudeMeasurement measurement, int droneConfiguration) {

    AttitudeState prediction; /* = Ax + Bu */
    AttitudeState innovation; /* = L (y - Cx) */

    /* Generated calculations for Kalman filter. */
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
            CALCULATE_QUAT_0(prediction);
            innovation.q[1] = $c1$i1;
            innovation.q[2] = $c1$i2;
            innovation.q[3] = $c1$i3;
            innovation.wx   = $c1$i4;
            innovation.wy   = $c1$i5;
            innovation.wz   = $c1$i6;
            innovation.nx   = $c1$i7;
            innovation.ny   = $c1$i8;
            innovation.nz   = $c1$i9;
            CALCULATE_QUAT_0(innovation);
            /* TODO: This last part is the same for all configurations */
            stateEstimate.q[0] = $c1$x0;
            stateEstimate.q[1] = $c1$x1;
            stateEstimate.q[2] = $c1$x2;
            stateEstimate.q[3] = $c1$x3;
            stateEstimate.wx   = $c1$x4;
            stateEstimate.wy   = $c1$x5;
            stateEstimate.wz   = $c1$x6;
            stateEstimate.nx   = $c1$x7;
            stateEstimate.ny   = $c1$x8;
            stateEstimate.nz   = $c1$x9;
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
            CALCULATE_QUAT_0(prediction);
            innovation.q[1] = $c2$i1;
            innovation.q[2] = $c2$i2;
            innovation.q[3] = $c2$i3;
            innovation.wx   = $c2$i4;
            innovation.wy   = $c2$i5;
            innovation.wz   = $c2$i6;
            innovation.nx   = $c2$i7;
            innovation.ny   = $c2$i8;
            innovation.nz   = $c2$i9;
            CALCULATE_QUAT_0(innovation);
            stateEstimate.q[0] = $c2$x0;
            stateEstimate.q[1] = $c2$x1;
            stateEstimate.q[2] = $c2$x2;
            stateEstimate.q[3] = $c2$x3;
            stateEstimate.wx   = $c2$x4;
            stateEstimate.wy   = $c2$x5;
            stateEstimate.wz   = $c2$x6;
            stateEstimate.nx   = $c2$x7;
            stateEstimate.ny   = $c2$x8;
            stateEstimate.nz   = $c2$x9;
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
            CALCULATE_QUAT_0(prediction);
            innovation.q[1] = $c3$i1;
            innovation.q[2] = $c3$i2;
            innovation.q[3] = $c3$i3;
            innovation.wx   = $c3$i4;
            innovation.wy   = $c3$i5;
            innovation.wz   = $c3$i6;
            innovation.nx   = $c3$i7;
            innovation.ny   = $c3$i8;
            innovation.nz   = $c3$i9;
            CALCULATE_QUAT_0(innovation);
            stateEstimate.q[0] = $c3$x0;
            stateEstimate.q[1] = $c3$x1;
            stateEstimate.q[2] = $c3$x2;
            stateEstimate.q[3] = $c3$x3;
            stateEstimate.wx   = $c3$x4;
            stateEstimate.wy   = $c3$x5;
            stateEstimate.wz   = $c3$x6;
            stateEstimate.nx   = $c3$x7;
            stateEstimate.ny   = $c3$x8;
            stateEstimate.nz   = $c3$x9;
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
            CALCULATE_QUAT_0(prediction);
            innovation.q[1] = $c4$i1;
            innovation.q[2] = $c4$i2;
            innovation.q[3] = $c4$i3;
            innovation.wx   = $c4$i4;
            innovation.wy   = $c4$i5;
            innovation.wz   = $c4$i6;
            innovation.nx   = $c4$i7;
            innovation.ny   = $c4$i8;
            innovation.nz   = $c4$i9;
            CALCULATE_QUAT_0(innovation);
            stateEstimate.q[0] = $c4$x0;
            stateEstimate.q[1] = $c4$x1;
            stateEstimate.q[2] = $c4$x2;
            stateEstimate.q[3] = $c4$x3;
            stateEstimate.wx   = $c4$x4;
            stateEstimate.wy   = $c4$x5;
            stateEstimate.wz   = $c4$x6;
            stateEstimate.nx   = $c4$x7;
            stateEstimate.ny   = $c4$x8;
            stateEstimate.nz   = $c4$x9;
            break;
    }
    return stateEstimate;
}
