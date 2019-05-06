#include <Altitude.hpp>
#include <math.h>   /* copysign, fabs */
#include <string.h> /* memcpy */

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AltitudeControlSignal AltitudeController::codegenControlSignal(
    AltitudeState stateEstimate, AltitudeReference reference,
    AltitudeIntegralWindup integralWindup, int droneConfiguration) {

    AltitudeControlSignal controlSignal;

    /* Set maximum integral windup. *
	 * Calculate controller output. */
    switch (droneConfiguration) {
        case 1: controlSignal.ut = $c1$u0; break;
        case 2: controlSignal.ut = $c2$u0; break;
        case 3: controlSignal.ut = $c3$u0; break;
        case 4: controlSignal.ut = $c4$u0; break;
        default: controlSignal = {};
    }
    return controlSignal;
}

/* Don't use integral action if tunerValue < 0.0. */
//if(tunerValue < 0.0)
//	integralWindup_max = 0.0;
//(void) tunerValue;

AltitudeIntegralWindup
AltitudeController::codegenIntegralWindup(AltitudeIntegralWindup integralWindup,
                                          AltitudeReference reference,
                                          AltitudeState stateEstimate,
                                          int droneConfiguration) {

    real_t maxIntegralWindup;

    switch (droneConfiguration) {
        case 1: maxIntegralWindup = $c1$maxWindup; break;
        case 2: maxIntegralWindup = $c2$maxWindup; break;
        case 3: maxIntegralWindup = $c3$maxWindup; break;
        case 4: maxIntegralWindup = $c4$maxWindup; break;
        default: maxIntegralWindup = 0.0;
    }

    integralWindup.z += $int0;
    if (fabs(integralWindup.z) > maxIntegralWindup)
        integralWindup.z = copysign(maxIntegralWindup, integralWindup.z);
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AltitudeState AltitudeController::codegenNextStateEstimate(AltitudeState stateEstimate,
                                       AltitudeControlSignal controlSignal,
                                       AltitudeMeasurement measurement,
                                       int droneConfiguration) {

    /*  vx, vy estimate: should be more than 1 sample for proper function */
    //stateEstimate[2] = (y[0] - stateEstimate[1]) / altTs;

    //stateEstimate[0] = 0.0;
    //stateEstimate[1] = y[0];

    // Trust model if the sonar returns 0 as height
    // TODO: what if sonar becomes detached and we don't receive any more data? then altitude will drift!
    if (measurement.z < 0.01)
        measurement.z = stateEstimate.z;

    //TODO:  wat is dit? 
    AltitudeState stateEstimateCopy;
    memcpy(stateEstimateCopy, stateEstimate, sizeof(stateEstimateCopy));

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
