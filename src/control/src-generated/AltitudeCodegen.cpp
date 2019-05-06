/* Automatically generated, using 
 *
 * Configuration 1: 
 * Q = 
 *   1.000000e-06 0 0
 *   0 8.000000e-01 0
 *   0 0 5.000000e-01
 * R = 30
 * I = 1.000000e-02
 *
 * Configuration 2: 
 * Q = 
 *   1.000000e-06 0 0
 *   0 8.000000e-01 0
 *   0 0 5.000000e-01
 * R = 30
 * I = 1.000000e-02
 *
 * Configuration 3: 
 * Q = 
 *   1.000000e-06 0 0
 *   0 8.000000e-01 0
 *   0 0 5.000000e-01
 * R = 30
 * I = 1.000000e-02
 *
 * Configuration 4: 
 * Q = 
 *   1.000000e-06 0 0
 *   0 8.000000e-01 0
 *   0 0 5.000000e-01
 * R = 30
 * I = 1.000000e-02
 *
 */

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

	/* Calculate controller output based on drone configuration. */
    AltitudeControlSignal controlSignal;
    switch (droneConfiguration) {
        case 1: controlSignal.ut = 0.14309672000000001*reference.z - 0.00147842*stateEstimate.nt - 0.14309672000000001*stateEstimate.z - 0.14969718000000001*stateEstimate.vz + 0.01*integralWindup.z; break;
        case 2: controlSignal.ut = 0.14309672000000001*reference.z - 0.00147842*stateEstimate.nt - 0.14309672000000001*stateEstimate.z - 0.14969718000000001*stateEstimate.vz + 0.01*integralWindup.z; break;
        case 3: controlSignal.ut = 0.14309672000000001*reference.z - 0.00147842*stateEstimate.nt - 0.14309672000000001*stateEstimate.z - 0.14969718000000001*stateEstimate.vz + 0.01*integralWindup.z; break;
        case 4: controlSignal.ut = 0.14309672000000001*reference.z - 0.00147842*stateEstimate.nt - 0.14309672000000001*stateEstimate.z - 0.14969718000000001*stateEstimate.vz + 0.01*integralWindup.z; break;
        default: controlSignal = {};
    }
    return controlSignal;
}

AltitudeIntegralWindup AltitudeController::codegenIntegralWindup(
    AltitudeIntegralWindup integralWindup, AltitudeReference reference,
    AltitudeState stateEstimate, int droneConfiguration) {

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
    integralWindup.z += 0.05*reference.z - 0.05*stateEstimate.z;
    if (fabs(integralWindup.z) > maxIntegralWindup)
        integralWindup.z = copysign(maxIntegralWindup, integralWindup.z);

    return integralWindup;
}

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
AltitudeState AltitudeController::codegenNextStateEstimate(
    AltitudeState stateEstimate, AltitudeControlSignal controlSignal,
    AltitudeMeasurement measurement, int droneConfiguration) {

    /* Trust model if the sonar returns 0 as height. */
    // TODO: what if sonar becomes detached and we don't receive any more data?
    // TODO: then altitude will drift!
    if (measurement.z < 0.01)
        measurement.z = stateEstimate.z;

    /* Calculate next state using Kalman Filter based on drone configuration. */
    //TODO: maybe make this newStateEstimate (nitpicky...)
    AltitudeState stateEstimateCopy = stateEstimate;
    switch (droneConfiguration) {
        case 1:
            stateEstimate.nt = 88.61867170271104*controlSignal.ut + 0.00141277*measurement.z + 0.23965103644177566*stateEstimateCopy.nt - 0.00141277*stateEstimateCopy.z;
            stateEstimate.z  = 0.015060059336428257*controlSignal.ut + 0.99976927000000004*measurement.z + 0.00024516798706424579*stateEstimateCopy.nt + 0.0002307299999999568*stateEstimateCopy.z + 0.05*stateEstimateCopy.vz;
            stateEstimate.vz = 0.81640939692393877*controlSignal.ut + 2.6854483099999999*measurement.z + 0.0079705373850425167*stateEstimateCopy.nt - 2.6854483099999999*stateEstimateCopy.z + stateEstimateCopy.vz;
            break;
        case 2:
            stateEstimate.nt = 88.61867170271104*controlSignal.ut + 0.00141277*measurement.z + 0.23965103644177566*stateEstimateCopy.nt - 0.00141277*stateEstimateCopy.z;
            stateEstimate.z  = 0.015060059336428257*controlSignal.ut + 0.99976927000000004*measurement.z + 0.00024516798706424579*stateEstimateCopy.nt + 0.0002307299999999568*stateEstimateCopy.z + 0.05*stateEstimateCopy.vz;
            stateEstimate.vz = 0.81640939692393877*controlSignal.ut + 2.6854483099999999*measurement.z + 0.0079705373850425167*stateEstimateCopy.nt - 2.6854483099999999*stateEstimateCopy.z + stateEstimateCopy.vz;
            break;
        case 3:
            stateEstimate.nt = 88.61867170271104*controlSignal.ut + 0.00141277*measurement.z + 0.23965103644177566*stateEstimateCopy.nt - 0.00141277*stateEstimateCopy.z;
            stateEstimate.z  = 0.015060059336428257*controlSignal.ut + 0.99976927000000004*measurement.z + 0.00024516798706424579*stateEstimateCopy.nt + 0.0002307299999999568*stateEstimateCopy.z + 0.05*stateEstimateCopy.vz;
            stateEstimate.vz = 0.81640939692393877*controlSignal.ut + 2.6854483099999999*measurement.z + 0.0079705373850425167*stateEstimateCopy.nt - 2.6854483099999999*stateEstimateCopy.z + stateEstimateCopy.vz;
            break;
        case 4:
            stateEstimate.nt = 88.61867170271104*controlSignal.ut + 0.00141277*measurement.z + 0.23965103644177566*stateEstimateCopy.nt - 0.00141277*stateEstimateCopy.z;
            stateEstimate.z  = 0.015060059336428257*controlSignal.ut + 0.99976927000000004*measurement.z + 0.00024516798706424579*stateEstimateCopy.nt + 0.0002307299999999568*stateEstimateCopy.z + 0.05*stateEstimateCopy.vz;
            stateEstimate.vz = 0.81640939692393877*controlSignal.ut + 2.6854483099999999*measurement.z + 0.0079705373850425167*stateEstimateCopy.nt - 2.6854483099999999*stateEstimateCopy.z + stateEstimateCopy.vz;
            break;
    }
    return stateEstimate;
}
