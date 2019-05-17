#include <Altitude.hpp>
#include <math.h>   /* copysign, fabs */
#include <string.h> /* memcpy */

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AltitudeControlSignal
AltitudeController::codegenControlSignal(AltitudeState stateEstimate,
                                         AltitudeReference reference,
                                         AltitudeIntegralWindup integralWindup,
                                         int droneConfiguration) {

	/* Calculate controller output based on drone configuration. */
    AltitudeControlSignal controlSignal;
    switch (droneConfiguration) {
        case 1: controlSignal.ut = $c1$u0; break;
        case 2: controlSignal.ut = $c2$u0; break;
        case 3: controlSignal.ut = $c3$u0; break;
        case 4: controlSignal.ut = $c4$u0; break;
        default: controlSignal = {};
    }
    (void)integralWindup;
    return controlSignal;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AltitudeIntegralWindup
AltitudeController::codegenIntegralWindup(AltitudeIntegralWindup integralWindup,
                                          AltitudeReference reference,
                                          AltitudeState stateEstimate,
                                          int droneConfiguration) {

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
    integralWindup.z += $int0;
    if (fabs(integralWindup.z) > maxIntegralWindup)
        integralWindup.z = copysign(maxIntegralWindup, integralWindup.z);

    return integralWindup;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AltitudeState
AltitudeController::codegenNextStateEstimate(AltitudeState stateEstimate,
                                             AltitudeControlSignal controlSignal,
                                             AltitudeMeasurement measurement,
                                             int droneConfiguration) {

    /* Trust model if the sonar returns 0 as height. */
    if (measurement.z < 0.01)
        measurement.z = stateEstimate.z;

    /* Calculate next state using Kalman Filter based on drone configuration. */
    AltitudeState stateEstimateCopy = stateEstimate;
    switch (droneConfiguration) {
        case 1:
            stateEstimate.nt = $c1$x0;
            stateEstimate.z  = $c1$x1;
            stateEstimate.vz = $c1$x2;
            break;
        case 2:
            stateEstimate.nt = $c2$x0;
            stateEstimate.z  = $c2$x1;
            stateEstimate.vz = $c2$x2;
            break;
        case 3:
            stateEstimate.nt = $c3$x0;
            stateEstimate.z  = $c3$x1;
            stateEstimate.vz = $c3$x2;
            break;
        case 4:
            stateEstimate.nt = $c4$x0;
            stateEstimate.z  = $c4$x1;
            stateEstimate.vz = $c4$x2;
            break;
    }
    return stateEstimate;
}
