/* Automatically generated, using... 
 *
 * Configuration 1: 
 * Q = 
 *   1.396245e+02 0 0 0 0 0 0 0 0
 *   0 1.396245e+02 0 0 0 0 0 0 0
 *   0 0 1.528118e+01 0 0 0 0 0 0
 *   0 0 0 1.150520e+00 0 0 0 0 0
 *   0 0 0 0 1.150520e+00 0 0 0 0
 *   0 0 0 0 0 1.209919e-01 0 0 0
 *   0 0 0 0 0 0 9.976476e-08 0 0
 *   0 0 0 0 0 0 0 9.976476e-08 0
 *   0 0 0 0 0 0 0 0 9.976476e-09
 * R = 
 *   24 0 0
 *   0 24 0
 *   0 0 24
 *
 * Configuration 2: 
 * Q = 
 *   1.396245e+02 0 0 0 0 0 0 0 0
 *   0 1.396245e+02 0 0 0 0 0 0 0
 *   0 0 1.528118e+01 0 0 0 0 0 0
 *   0 0 0 1.150520e+00 0 0 0 0 0
 *   0 0 0 0 1.150520e+00 0 0 0 0
 *   0 0 0 0 0 1.209919e-01 0 0 0
 *   0 0 0 0 0 0 9.976476e-08 0 0
 *   0 0 0 0 0 0 0 9.976476e-08 0
 *   0 0 0 0 0 0 0 0 9.976476e-09
 * R = 
 *   24 0 0
 *   0 24 0
 *   0 0 24
 *
 * Configuration 3: 
 * Q = 
 *   1.396245e+02 0 0 0 0 0 0 0 0
 *   0 1.396245e+02 0 0 0 0 0 0 0
 *   0 0 1.528118e+01 0 0 0 0 0 0
 *   0 0 0 1.150520e+00 0 0 0 0 0
 *   0 0 0 0 1.150520e+00 0 0 0 0
 *   0 0 0 0 0 1.209919e-01 0 0 0
 *   0 0 0 0 0 0 9.976476e-08 0 0
 *   0 0 0 0 0 0 0 9.976476e-08 0
 *   0 0 0 0 0 0 0 0 9.976476e-09
 * R = 
 *   24 0 0
 *   0 24 0
 *   0 0 24
 *
 * Configuration 4: 
 * Q = 
 *   1.396245e+02 0 0 0 0 0 0 0 0
 *   0 1.396245e+02 0 0 0 0 0 0 0
 *   0 0 1.528118e+01 0 0 0 0 0 0
 *   0 0 0 1.150520e+00 0 0 0 0 0
 *   0 0 0 0 1.150520e+00 0 0 0 0
 *   0 0 0 0 0 1.209919e-01 0 0 0
 *   0 0 0 0 0 0 9.976476e-08 0 0
 *   0 0 0 0 0 0 0 9.976476e-08 0
 *   0 0 0 0 0 0 0 0 9.976476e-09
 * R = 
 *   24 0 0
 *   0 24 0
 *   0 0 24
 *
 */

#include <Attitude.hpp>
#include <math.h> /* sqrt */

#define SQ(x) ((x) * (x))
#define QUAT_0(x) (x.q[0] = sqrt(1.0 - SQ(x.q[1]) - SQ(x.q[2]) - SQ(x.q[3])))

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
            controlSignal.ux = 0.8*integralWindup.q1 - 0.014676*stateEstimate.nx - 0.23209235*stateEstimate.wx - 2.1641921399999999*reference.q[0]*stateEstimate.q[1] + 2.1641921399999999*reference.q[1]*stateEstimate.q[0] - 2.1641921399999999*reference.q[2]*stateEstimate.q[3] + 2.1641921399999999*reference.q[3]*stateEstimate.q[2];
            controlSignal.uy = 0.8*integralWindup.q2 - 0.01418606*stateEstimate.ny - 0.23406505*stateEstimate.wy - 2.17247061*reference.q[0]*stateEstimate.q[2] + 2.17247061*reference.q[2]*stateEstimate.q[0] + 2.17247061*reference.q[1]*stateEstimate.q[3] - 2.17247061*reference.q[3]*stateEstimate.q[1];
            controlSignal.uz = 0.0062020900000000004*stateEstimate.q[0] - 0.12156138*stateEstimate.wz - 0.78161983000000002*reference.q[0]*stateEstimate.q[3] - 0.78161983000000002*reference.q[1]*stateEstimate.q[2] + 0.78161983000000002*reference.q[2]*stateEstimate.q[1] + 0.78161983000000002*reference.q[3]*stateEstimate.q[0];
            break;
        case 2:
            controlSignal.ux = 0.8*integralWindup.q1 - 0.014676*stateEstimate.nx - 0.23209235*stateEstimate.wx - 2.1641921399999999*reference.q[0]*stateEstimate.q[1] + 2.1641921399999999*reference.q[1]*stateEstimate.q[0] - 2.1641921399999999*reference.q[2]*stateEstimate.q[3] + 2.1641921399999999*reference.q[3]*stateEstimate.q[2];
            controlSignal.uy = 0.8*integralWindup.q2 - 0.01418606*stateEstimate.ny - 0.23406505*stateEstimate.wy - 2.17247061*reference.q[0]*stateEstimate.q[2] + 2.17247061*reference.q[2]*stateEstimate.q[0] + 2.17247061*reference.q[1]*stateEstimate.q[3] - 2.17247061*reference.q[3]*stateEstimate.q[1];
            controlSignal.uz = 0.0062020900000000004*stateEstimate.q[0] - 0.12156138*stateEstimate.wz - 0.78161983000000002*reference.q[0]*stateEstimate.q[3] - 0.78161983000000002*reference.q[1]*stateEstimate.q[2] + 0.78161983000000002*reference.q[2]*stateEstimate.q[1] + 0.78161983000000002*reference.q[3]*stateEstimate.q[0];
            break;
        case 3:
            controlSignal.ux = 0.8*integralWindup.q1 - 0.014676*stateEstimate.nx - 0.23209235*stateEstimate.wx - 2.1641921399999999*reference.q[0]*stateEstimate.q[1] + 2.1641921399999999*reference.q[1]*stateEstimate.q[0] - 2.1641921399999999*reference.q[2]*stateEstimate.q[3] + 2.1641921399999999*reference.q[3]*stateEstimate.q[2];
            controlSignal.uy = 0.8*integralWindup.q2 - 0.01418606*stateEstimate.ny - 0.23406505*stateEstimate.wy - 2.17247061*reference.q[0]*stateEstimate.q[2] + 2.17247061*reference.q[2]*stateEstimate.q[0] + 2.17247061*reference.q[1]*stateEstimate.q[3] - 2.17247061*reference.q[3]*stateEstimate.q[1];
            controlSignal.uz = 0.0062020900000000004*stateEstimate.q[0] - 0.12156138*stateEstimate.wz - 0.78161983000000002*reference.q[0]*stateEstimate.q[3] - 0.78161983000000002*reference.q[1]*stateEstimate.q[2] + 0.78161983000000002*reference.q[2]*stateEstimate.q[1] + 0.78161983000000002*reference.q[3]*stateEstimate.q[0];
            break;
        case 4:
            controlSignal.ux = 0.8*integralWindup.q1 - 0.014676*stateEstimate.nx - 0.23209235*stateEstimate.wx - 2.1641921399999999*reference.q[0]*stateEstimate.q[1] + 2.1641921399999999*reference.q[1]*stateEstimate.q[0] - 2.1641921399999999*reference.q[2]*stateEstimate.q[3] + 2.1641921399999999*reference.q[3]*stateEstimate.q[2];
            controlSignal.uy = 0.8*integralWindup.q2 - 0.01418606*stateEstimate.ny - 0.23406505*stateEstimate.wy - 2.17247061*reference.q[0]*stateEstimate.q[2] + 2.17247061*reference.q[2]*stateEstimate.q[0] + 2.17247061*reference.q[1]*stateEstimate.q[3] - 2.17247061*reference.q[3]*stateEstimate.q[1];
            controlSignal.uz = 0.0062020900000000004*stateEstimate.q[0] - 0.12156138*stateEstimate.wz - 0.78161983000000002*reference.q[0]*stateEstimate.q[3] - 0.78161983000000002*reference.q[1]*stateEstimate.q[2] + 0.78161983000000002*reference.q[2]*stateEstimate.q[1] + 0.78161983000000002*reference.q[3]*stateEstimate.q[0];
            break;
        default: controlSignal = {};
    }
    return controlSignal;
}

AttitudeIntegralWindup AttitudeController::codegenIntegralWindup(
    AttitudeIntegralWindup integralWindup, AttitudeReference reference,
    AttitudeState stateEstimate, int droneConfiguration) {

    /* Set maximum integral windup based on drone configuration. */
    real_t maxIntegralWindup;
    switch (droneConfiguration) {
        case 1: maxIntegralWindup = 10; break;
        case 2: maxIntegralWindup = 10; break;
        case 3: maxIntegralWindup = 10; break;
        case 4: maxIntegralWindup = 10; break;
        default: maxIntegralWindup = 0.0;
    }

    /* Update integral windup. */
    integralWindup.q1 += 0.0042016806722689076*reference.q[1] - 0.0042016806722689076*stateEstimate.q[1];
    integralWindup.q2 += 0.0042016806722689076*reference.q[2] - 0.0042016806722689076*stateEstimate.q[2];
    integralWindup.q3 += 0.0042016806722689076*reference.q[3] - 0.0042016806722689076*stateEstimate.q[3];
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

    /* Calculate next state using Kalman Filter based on drone configuration. */
    AttitudeState prediction; /* = Ax + Bu */
    AttitudeState innovation; /* = L (y - Cx) */
    switch (droneConfiguration) {
        case 1:
            prediction.q[1] = 0.000065847107510608714*controlSignal.ux + stateEstimate.q[1] + 0.0021008403361344538*stateEstimate.wx + 0.000013979844768739008*stateEstimate.nx;
            prediction.q[2] = 0.000062167416208545279*controlSignal.uy + stateEstimate.q[2] + 0.0021008403361344538*stateEstimate.wy + 0.000013198618149309471*stateEstimate.ny;
            prediction.q[3] = 0.0010533908709510554*controlSignal.uz + stateEstimate.q[3] + 0.0021008403361344538*stateEstimate.wz - 0.0000082128410598254026*stateEstimate.q[0];
            prediction.wx   = 0.093105766159801776*controlSignal.ux + stateEstimate.wx + 0.013047814194874266*stateEstimate.nx;
            prediction.wy   = 0.087902796874401085*controlSignal.uy + stateEstimate.wy + 0.012318671636925406*stateEstimate.ny;
            prediction.wz   = 0.98495745014468838*controlSignal.uz + stateEstimate.wz - 0.0076652942813972298*stateEstimate.q[0];
            prediction.nx   = 13.184386754639419*controlSignal.ux + 0.8868778485230423*stateEstimate.nx;
            prediction.ny   = 13.184386754639419*controlSignal.uy + 0.8868778485230423*stateEstimate.ny;
            prediction.nz   = 13.184386754639421*controlSignal.uz + 0.8868778485230423*stateEstimate.q[0];
            QUAT_0(prediction);
            innovation.q[1] = 0.00000022000000000000001*measurement.wx - 0.00000022000000000000001*stateEstimate.wx - 0.99019513999999997*measurement.q[0]*stateEstimate.q[1] + 0.99019513999999997*measurement.q[1]*stateEstimate.q[0] - 0.99019513999999997*measurement.q[4]*stateEstimate.q[3] + 0.99019513999999997*measurement.q[5]*stateEstimate.q[2];
            innovation.q[2] = 0.00000022000000000000001*measurement.wy - 0.00000022000000000000001*stateEstimate.wy - 0.99019513999999997*measurement.q[0]*stateEstimate.q[2] + 0.99019513999999997*measurement.q[4]*stateEstimate.q[0] + 0.99019513999999997*measurement.q[1]*stateEstimate.q[3] - 0.99019513999999997*measurement.q[5]*stateEstimate.q[1];
            innovation.q[3] = 0.00000029999999999999999*measurement.wz - 0.00000029999999999999999*stateEstimate.wz - 0.99019513999999997*measurement.q[0]*stateEstimate.q[3] - 0.99019513999999997*measurement.q[1]*stateEstimate.q[2] + 0.99019513999999997*measurement.q[4]*stateEstimate.q[1] + 0.99019513999999997*measurement.q[5]*stateEstimate.q[0];
            innovation.wx   = 0.99021711999999995*measurement.wx - 0.99021711999999995*stateEstimate.wx - 0.00000022000000000000001*measurement.q[0]*stateEstimate.q[1] + 0.00000022000000000000001*measurement.q[1]*stateEstimate.q[0] - 0.00000022000000000000001*measurement.q[4]*stateEstimate.q[3] + 0.00000022000000000000001*measurement.q[5]*stateEstimate.q[2];
            innovation.wy   = 0.99021475000000003*measurement.wy - 0.99021475000000003*stateEstimate.wy - 0.00000022000000000000001*measurement.q[0]*stateEstimate.q[2] + 0.00000022000000000000001*measurement.q[4]*stateEstimate.q[0] + 0.00000022000000000000001*measurement.q[1]*stateEstimate.q[3] - 0.00000022000000000000001*measurement.q[5]*stateEstimate.q[1];
            innovation.wz   = 0.99029456999999999*measurement.wz - 0.99029456999999999*stateEstimate.wz - 0.00000029999999999999999*measurement.q[0]*stateEstimate.q[3] - 0.00000029999999999999999*measurement.q[1]*stateEstimate.q[2] + 0.00000029999999999999999*measurement.q[4]*stateEstimate.q[1] + 0.00000029999999999999999*measurement.q[5]*stateEstimate.q[0];
            innovation.nx   = 0.15726204999999999*measurement.wx - 0.15726204999999999*stateEstimate.wx - 0.00016373000000000001*measurement.q[0]*stateEstimate.q[1] + 0.00016373000000000001*measurement.q[1]*stateEstimate.q[0] - 0.00016373000000000001*measurement.q[4]*stateEstimate.q[3] + 0.00016373000000000001*measurement.q[5]*stateEstimate.q[2];
            innovation.ny   = 0.14864647*measurement.wy - 0.14864647*stateEstimate.wy - 0.00015477000000000001*measurement.q[0]*stateEstimate.q[2] + 0.00015477000000000001*measurement.q[4]*stateEstimate.q[0] + 0.00015477000000000001*measurement.q[1]*stateEstimate.q[3] - 0.00015477000000000001*measurement.q[5]*stateEstimate.q[1];
            innovation.nz   = 0.041807249999999997*measurement.wz - 0.041807249999999997*stateEstimate.wz - 0.000044450000000000003*measurement.q[0]*stateEstimate.q[3] - 0.000044450000000000003*measurement.q[1]*stateEstimate.q[2] + 0.000044450000000000003*measurement.q[4]*stateEstimate.q[1] + 0.000044450000000000003*measurement.q[5]*stateEstimate.q[0];
            QUAT_0(innovation);
            // TODO: This last part is the same for all configurations
            stateEstimate.q[0] = prediction.q[0]*innovation.q[0] - 1.0*prediction.q[1]*innovation.q[1] - 1.0*prediction.q[2]*innovation.q[2] - 1.0*prediction.q[3]*innovation.q[3];
            stateEstimate.q[1] = prediction.q[0]*innovation.q[1] + prediction.q[1]*innovation.q[0] + prediction.q[2]*innovation.q[3] - 1.0*prediction.q[3]*innovation.q[2];
            stateEstimate.q[2] = prediction.q[0]*innovation.q[2] + prediction.q[2]*innovation.q[0] - 1.0*prediction.q[1]*innovation.q[3] + prediction.q[3]*innovation.q[1];
            stateEstimate.q[3] = prediction.q[0]*innovation.q[3] + prediction.q[1]*innovation.q[2] - 1.0*prediction.q[2]*innovation.q[1] + prediction.q[3]*innovation.q[0];
            stateEstimate.wx   = prediction.wx + innovation.wx;
            stateEstimate.wy   = prediction.wy + innovation.wy;
            stateEstimate.wz   = prediction.wz + innovation.wz;
            stateEstimate.nx   = prediction.nx + innovation.nx;
            stateEstimate.ny   = prediction.ny + innovation.ny;
            stateEstimate.nz   = prediction.q[0] + innovation.q[0];
            break;
        case 2:
            prediction.q[1] = 0.000065847107510608714*controlSignal.ux + stateEstimate.q[1] + 0.0021008403361344538*stateEstimate.wx + 0.000013979844768739008*stateEstimate.nx;
            prediction.q[2] = 0.000062167416208545279*controlSignal.uy + stateEstimate.q[2] + 0.0021008403361344538*stateEstimate.wy + 0.000013198618149309471*stateEstimate.ny;
            prediction.q[3] = 0.0010533908709510554*controlSignal.uz + stateEstimate.q[3] + 0.0021008403361344538*stateEstimate.wz - 0.0000082128410598254026*stateEstimate.q[0];
            prediction.wx   = 0.093105766159801776*controlSignal.ux + stateEstimate.wx + 0.013047814194874266*stateEstimate.nx;
            prediction.wy   = 0.087902796874401085*controlSignal.uy + stateEstimate.wy + 0.012318671636925406*stateEstimate.ny;
            prediction.wz   = 0.98495745014468838*controlSignal.uz + stateEstimate.wz - 0.0076652942813972298*stateEstimate.q[0];
            prediction.nx   = 13.184386754639419*controlSignal.ux + 0.8868778485230423*stateEstimate.nx;
            prediction.ny   = 13.184386754639419*controlSignal.uy + 0.8868778485230423*stateEstimate.ny;
            prediction.nz   = 13.184386754639421*controlSignal.uz + 0.8868778485230423*stateEstimate.q[0];
            QUAT_0(prediction);
            innovation.q[1] = 0.00000022000000000000001*measurement.wx - 0.00000022000000000000001*stateEstimate.wx - 0.99019513999999997*measurement.q[0]*stateEstimate.q[1] + 0.99019513999999997*measurement.q[1]*stateEstimate.q[0] - 0.99019513999999997*measurement.q[4]*stateEstimate.q[3] + 0.99019513999999997*measurement.q[5]*stateEstimate.q[2];
            innovation.q[2] = 0.00000022000000000000001*measurement.wy - 0.00000022000000000000001*stateEstimate.wy - 0.99019513999999997*measurement.q[0]*stateEstimate.q[2] + 0.99019513999999997*measurement.q[4]*stateEstimate.q[0] + 0.99019513999999997*measurement.q[1]*stateEstimate.q[3] - 0.99019513999999997*measurement.q[5]*stateEstimate.q[1];
            innovation.q[3] = 0.00000029999999999999999*measurement.wz - 0.00000029999999999999999*stateEstimate.wz - 0.99019513999999997*measurement.q[0]*stateEstimate.q[3] - 0.99019513999999997*measurement.q[1]*stateEstimate.q[2] + 0.99019513999999997*measurement.q[4]*stateEstimate.q[1] + 0.99019513999999997*measurement.q[5]*stateEstimate.q[0];
            innovation.wx   = 0.99021711999999995*measurement.wx - 0.99021711999999995*stateEstimate.wx - 0.00000022000000000000001*measurement.q[0]*stateEstimate.q[1] + 0.00000022000000000000001*measurement.q[1]*stateEstimate.q[0] - 0.00000022000000000000001*measurement.q[4]*stateEstimate.q[3] + 0.00000022000000000000001*measurement.q[5]*stateEstimate.q[2];
            innovation.wy   = 0.99021475000000003*measurement.wy - 0.99021475000000003*stateEstimate.wy - 0.00000022000000000000001*measurement.q[0]*stateEstimate.q[2] + 0.00000022000000000000001*measurement.q[4]*stateEstimate.q[0] + 0.00000022000000000000001*measurement.q[1]*stateEstimate.q[3] - 0.00000022000000000000001*measurement.q[5]*stateEstimate.q[1];
            innovation.wz   = 0.99029456999999999*measurement.wz - 0.99029456999999999*stateEstimate.wz - 0.00000029999999999999999*measurement.q[0]*stateEstimate.q[3] - 0.00000029999999999999999*measurement.q[1]*stateEstimate.q[2] + 0.00000029999999999999999*measurement.q[4]*stateEstimate.q[1] + 0.00000029999999999999999*measurement.q[5]*stateEstimate.q[0];
            innovation.nx   = 0.15726204999999999*measurement.wx - 0.15726204999999999*stateEstimate.wx - 0.00016373000000000001*measurement.q[0]*stateEstimate.q[1] + 0.00016373000000000001*measurement.q[1]*stateEstimate.q[0] - 0.00016373000000000001*measurement.q[4]*stateEstimate.q[3] + 0.00016373000000000001*measurement.q[5]*stateEstimate.q[2];
            innovation.ny   = 0.14864647*measurement.wy - 0.14864647*stateEstimate.wy - 0.00015477000000000001*measurement.q[0]*stateEstimate.q[2] + 0.00015477000000000001*measurement.q[4]*stateEstimate.q[0] + 0.00015477000000000001*measurement.q[1]*stateEstimate.q[3] - 0.00015477000000000001*measurement.q[5]*stateEstimate.q[1];
            innovation.nz   = 0.041807249999999997*measurement.wz - 0.041807249999999997*stateEstimate.wz - 0.000044450000000000003*measurement.q[0]*stateEstimate.q[3] - 0.000044450000000000003*measurement.q[1]*stateEstimate.q[2] + 0.000044450000000000003*measurement.q[4]*stateEstimate.q[1] + 0.000044450000000000003*measurement.q[5]*stateEstimate.q[0];
            QUAT_0(innovation);
            stateEstimate.q[0] = prediction.q[0]*innovation.q[0] - 1.0*prediction.q[1]*innovation.q[1] - 1.0*prediction.q[2]*innovation.q[2] - 1.0*prediction.q[3]*innovation.q[3];
            stateEstimate.q[1] = prediction.q[0]*innovation.q[1] + prediction.q[1]*innovation.q[0] + prediction.q[2]*innovation.q[3] - 1.0*prediction.q[3]*innovation.q[2];
            stateEstimate.q[2] = prediction.q[0]*innovation.q[2] + prediction.q[2]*innovation.q[0] - 1.0*prediction.q[1]*innovation.q[3] + prediction.q[3]*innovation.q[1];
            stateEstimate.q[3] = prediction.q[0]*innovation.q[3] + prediction.q[1]*innovation.q[2] - 1.0*prediction.q[2]*innovation.q[1] + prediction.q[3]*innovation.q[0];
            stateEstimate.wx   = prediction.wx + innovation.wx;
            stateEstimate.wy   = prediction.wy + innovation.wy;
            stateEstimate.wz   = prediction.wz + innovation.wz;
            stateEstimate.nx   = prediction.nx + innovation.nx;
            stateEstimate.ny   = prediction.ny + innovation.ny;
            stateEstimate.nz   = prediction.q[0] + innovation.q[0];
            break;
        case 3:
            prediction.q[1] = 0.000065847107510608714*controlSignal.ux + stateEstimate.q[1] + 0.0021008403361344538*stateEstimate.wx + 0.000013979844768739008*stateEstimate.nx;
            prediction.q[2] = 0.000062167416208545279*controlSignal.uy + stateEstimate.q[2] + 0.0021008403361344538*stateEstimate.wy + 0.000013198618149309471*stateEstimate.ny;
            prediction.q[3] = 0.0010533908709510554*controlSignal.uz + stateEstimate.q[3] + 0.0021008403361344538*stateEstimate.wz - 0.0000082128410598254026*stateEstimate.q[0];
            prediction.wx   = 0.093105766159801776*controlSignal.ux + stateEstimate.wx + 0.013047814194874266*stateEstimate.nx;
            prediction.wy   = 0.087902796874401085*controlSignal.uy + stateEstimate.wy + 0.012318671636925406*stateEstimate.ny;
            prediction.wz   = 0.98495745014468838*controlSignal.uz + stateEstimate.wz - 0.0076652942813972298*stateEstimate.q[0];
            prediction.nx   = 13.184386754639419*controlSignal.ux + 0.8868778485230423*stateEstimate.nx;
            prediction.ny   = 13.184386754639419*controlSignal.uy + 0.8868778485230423*stateEstimate.ny;
            prediction.nz   = 13.184386754639421*controlSignal.uz + 0.8868778485230423*stateEstimate.q[0];
            QUAT_0(prediction);
            innovation.q[1] = 0.00000022000000000000001*measurement.wx - 0.00000022000000000000001*stateEstimate.wx - 0.99019513999999997*measurement.q[0]*stateEstimate.q[1] + 0.99019513999999997*measurement.q[1]*stateEstimate.q[0] - 0.99019513999999997*measurement.q[4]*stateEstimate.q[3] + 0.99019513999999997*measurement.q[5]*stateEstimate.q[2];
            innovation.q[2] = 0.00000022000000000000001*measurement.wy - 0.00000022000000000000001*stateEstimate.wy - 0.99019513999999997*measurement.q[0]*stateEstimate.q[2] + 0.99019513999999997*measurement.q[4]*stateEstimate.q[0] + 0.99019513999999997*measurement.q[1]*stateEstimate.q[3] - 0.99019513999999997*measurement.q[5]*stateEstimate.q[1];
            innovation.q[3] = 0.00000029999999999999999*measurement.wz - 0.00000029999999999999999*stateEstimate.wz - 0.99019513999999997*measurement.q[0]*stateEstimate.q[3] - 0.99019513999999997*measurement.q[1]*stateEstimate.q[2] + 0.99019513999999997*measurement.q[4]*stateEstimate.q[1] + 0.99019513999999997*measurement.q[5]*stateEstimate.q[0];
            innovation.wx   = 0.99021711999999995*measurement.wx - 0.99021711999999995*stateEstimate.wx - 0.00000022000000000000001*measurement.q[0]*stateEstimate.q[1] + 0.00000022000000000000001*measurement.q[1]*stateEstimate.q[0] - 0.00000022000000000000001*measurement.q[4]*stateEstimate.q[3] + 0.00000022000000000000001*measurement.q[5]*stateEstimate.q[2];
            innovation.wy   = 0.99021475000000003*measurement.wy - 0.99021475000000003*stateEstimate.wy - 0.00000022000000000000001*measurement.q[0]*stateEstimate.q[2] + 0.00000022000000000000001*measurement.q[4]*stateEstimate.q[0] + 0.00000022000000000000001*measurement.q[1]*stateEstimate.q[3] - 0.00000022000000000000001*measurement.q[5]*stateEstimate.q[1];
            innovation.wz   = 0.99029456999999999*measurement.wz - 0.99029456999999999*stateEstimate.wz - 0.00000029999999999999999*measurement.q[0]*stateEstimate.q[3] - 0.00000029999999999999999*measurement.q[1]*stateEstimate.q[2] + 0.00000029999999999999999*measurement.q[4]*stateEstimate.q[1] + 0.00000029999999999999999*measurement.q[5]*stateEstimate.q[0];
            innovation.nx   = 0.15726204999999999*measurement.wx - 0.15726204999999999*stateEstimate.wx - 0.00016373000000000001*measurement.q[0]*stateEstimate.q[1] + 0.00016373000000000001*measurement.q[1]*stateEstimate.q[0] - 0.00016373000000000001*measurement.q[4]*stateEstimate.q[3] + 0.00016373000000000001*measurement.q[5]*stateEstimate.q[2];
            innovation.ny   = 0.14864647*measurement.wy - 0.14864647*stateEstimate.wy - 0.00015477000000000001*measurement.q[0]*stateEstimate.q[2] + 0.00015477000000000001*measurement.q[4]*stateEstimate.q[0] + 0.00015477000000000001*measurement.q[1]*stateEstimate.q[3] - 0.00015477000000000001*measurement.q[5]*stateEstimate.q[1];
            innovation.nz   = 0.041807249999999997*measurement.wz - 0.041807249999999997*stateEstimate.wz - 0.000044450000000000003*measurement.q[0]*stateEstimate.q[3] - 0.000044450000000000003*measurement.q[1]*stateEstimate.q[2] + 0.000044450000000000003*measurement.q[4]*stateEstimate.q[1] + 0.000044450000000000003*measurement.q[5]*stateEstimate.q[0];
            QUAT_0(innovation);
            stateEstimate.q[0] = prediction.q[0]*innovation.q[0] - 1.0*prediction.q[1]*innovation.q[1] - 1.0*prediction.q[2]*innovation.q[2] - 1.0*prediction.q[3]*innovation.q[3];
            stateEstimate.q[1] = prediction.q[0]*innovation.q[1] + prediction.q[1]*innovation.q[0] + prediction.q[2]*innovation.q[3] - 1.0*prediction.q[3]*innovation.q[2];
            stateEstimate.q[2] = prediction.q[0]*innovation.q[2] + prediction.q[2]*innovation.q[0] - 1.0*prediction.q[1]*innovation.q[3] + prediction.q[3]*innovation.q[1];
            stateEstimate.q[3] = prediction.q[0]*innovation.q[3] + prediction.q[1]*innovation.q[2] - 1.0*prediction.q[2]*innovation.q[1] + prediction.q[3]*innovation.q[0];
            stateEstimate.wx   = prediction.wx + innovation.wx;
            stateEstimate.wy   = prediction.wy + innovation.wy;
            stateEstimate.wz   = prediction.wz + innovation.wz;
            stateEstimate.nx   = prediction.nx + innovation.nx;
            stateEstimate.ny   = prediction.ny + innovation.ny;
            stateEstimate.nz   = prediction.q[0] + innovation.q[0];
            break;
        case 4:
            prediction.q[1] = 0.000065847107510608714*controlSignal.ux + stateEstimate.q[1] + 0.0021008403361344538*stateEstimate.wx + 0.000013979844768739008*stateEstimate.nx;
            prediction.q[2] = 0.000062167416208545279*controlSignal.uy + stateEstimate.q[2] + 0.0021008403361344538*stateEstimate.wy + 0.000013198618149309471*stateEstimate.ny;
            prediction.q[3] = 0.0010533908709510554*controlSignal.uz + stateEstimate.q[3] + 0.0021008403361344538*stateEstimate.wz - 0.0000082128410598254026*stateEstimate.q[0];
            prediction.wx   = 0.093105766159801776*controlSignal.ux + stateEstimate.wx + 0.013047814194874266*stateEstimate.nx;
            prediction.wy   = 0.087902796874401085*controlSignal.uy + stateEstimate.wy + 0.012318671636925406*stateEstimate.ny;
            prediction.wz   = 0.98495745014468838*controlSignal.uz + stateEstimate.wz - 0.0076652942813972298*stateEstimate.q[0];
            prediction.nx   = 13.184386754639419*controlSignal.ux + 0.8868778485230423*stateEstimate.nx;
            prediction.ny   = 13.184386754639419*controlSignal.uy + 0.8868778485230423*stateEstimate.ny;
            prediction.nz   = 13.184386754639421*controlSignal.uz + 0.8868778485230423*stateEstimate.q[0];
            QUAT_0(prediction);
            innovation.q[1] = 0.00000022000000000000001*measurement.wx - 0.00000022000000000000001*stateEstimate.wx - 0.99019513999999997*measurement.q[0]*stateEstimate.q[1] + 0.99019513999999997*measurement.q[1]*stateEstimate.q[0] - 0.99019513999999997*measurement.q[4]*stateEstimate.q[3] + 0.99019513999999997*measurement.q[5]*stateEstimate.q[2];
            innovation.q[2] = 0.00000022000000000000001*measurement.wy - 0.00000022000000000000001*stateEstimate.wy - 0.99019513999999997*measurement.q[0]*stateEstimate.q[2] + 0.99019513999999997*measurement.q[4]*stateEstimate.q[0] + 0.99019513999999997*measurement.q[1]*stateEstimate.q[3] - 0.99019513999999997*measurement.q[5]*stateEstimate.q[1];
            innovation.q[3] = 0.00000029999999999999999*measurement.wz - 0.00000029999999999999999*stateEstimate.wz - 0.99019513999999997*measurement.q[0]*stateEstimate.q[3] - 0.99019513999999997*measurement.q[1]*stateEstimate.q[2] + 0.99019513999999997*measurement.q[4]*stateEstimate.q[1] + 0.99019513999999997*measurement.q[5]*stateEstimate.q[0];
            innovation.wx   = 0.99021711999999995*measurement.wx - 0.99021711999999995*stateEstimate.wx - 0.00000022000000000000001*measurement.q[0]*stateEstimate.q[1] + 0.00000022000000000000001*measurement.q[1]*stateEstimate.q[0] - 0.00000022000000000000001*measurement.q[4]*stateEstimate.q[3] + 0.00000022000000000000001*measurement.q[5]*stateEstimate.q[2];
            innovation.wy   = 0.99021475000000003*measurement.wy - 0.99021475000000003*stateEstimate.wy - 0.00000022000000000000001*measurement.q[0]*stateEstimate.q[2] + 0.00000022000000000000001*measurement.q[4]*stateEstimate.q[0] + 0.00000022000000000000001*measurement.q[1]*stateEstimate.q[3] - 0.00000022000000000000001*measurement.q[5]*stateEstimate.q[1];
            innovation.wz   = 0.99029456999999999*measurement.wz - 0.99029456999999999*stateEstimate.wz - 0.00000029999999999999999*measurement.q[0]*stateEstimate.q[3] - 0.00000029999999999999999*measurement.q[1]*stateEstimate.q[2] + 0.00000029999999999999999*measurement.q[4]*stateEstimate.q[1] + 0.00000029999999999999999*measurement.q[5]*stateEstimate.q[0];
            innovation.nx   = 0.15726204999999999*measurement.wx - 0.15726204999999999*stateEstimate.wx - 0.00016373000000000001*measurement.q[0]*stateEstimate.q[1] + 0.00016373000000000001*measurement.q[1]*stateEstimate.q[0] - 0.00016373000000000001*measurement.q[4]*stateEstimate.q[3] + 0.00016373000000000001*measurement.q[5]*stateEstimate.q[2];
            innovation.ny   = 0.14864647*measurement.wy - 0.14864647*stateEstimate.wy - 0.00015477000000000001*measurement.q[0]*stateEstimate.q[2] + 0.00015477000000000001*measurement.q[4]*stateEstimate.q[0] + 0.00015477000000000001*measurement.q[1]*stateEstimate.q[3] - 0.00015477000000000001*measurement.q[5]*stateEstimate.q[1];
            innovation.nz   = 0.041807249999999997*measurement.wz - 0.041807249999999997*stateEstimate.wz - 0.000044450000000000003*measurement.q[0]*stateEstimate.q[3] - 0.000044450000000000003*measurement.q[1]*stateEstimate.q[2] + 0.000044450000000000003*measurement.q[4]*stateEstimate.q[1] + 0.000044450000000000003*measurement.q[5]*stateEstimate.q[0];
            QUAT_0(innovation);
            stateEstimate.q[0] = prediction.q[0]*innovation.q[0] - 1.0*prediction.q[1]*innovation.q[1] - 1.0*prediction.q[2]*innovation.q[2] - 1.0*prediction.q[3]*innovation.q[3];
            stateEstimate.q[1] = prediction.q[0]*innovation.q[1] + prediction.q[1]*innovation.q[0] + prediction.q[2]*innovation.q[3] - 1.0*prediction.q[3]*innovation.q[2];
            stateEstimate.q[2] = prediction.q[0]*innovation.q[2] + prediction.q[2]*innovation.q[0] - 1.0*prediction.q[1]*innovation.q[3] + prediction.q[3]*innovation.q[1];
            stateEstimate.q[3] = prediction.q[0]*innovation.q[3] + prediction.q[1]*innovation.q[2] - 1.0*prediction.q[2]*innovation.q[1] + prediction.q[3]*innovation.q[0];
            stateEstimate.wx   = prediction.wx + innovation.wx;
            stateEstimate.wy   = prediction.wy + innovation.wy;
            stateEstimate.wz   = prediction.wz + innovation.wz;
            stateEstimate.nx   = prediction.nx + innovation.nx;
            stateEstimate.ny   = prediction.ny + innovation.ny;
            stateEstimate.nz   = prediction.q[0] + innovation.q[0];
            break;
    }
    return stateEstimate;
}
