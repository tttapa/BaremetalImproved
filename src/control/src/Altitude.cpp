// Original: BareMetal/src/control/altitude.c

#include <Altitude.hpp>
#include <Matrix.hpp>
#include <TiltCorrection.hpp>
#include <configuration.hpp>

void AltitudeController::updateObserver(AltitudeMeasurement measurement) {

    //TODO: measurement flags
    if (*NEW_HEIGHT_MEASUREMENT_FLAG == 1) {

        updateObserverCodegen(AltitudeController::stateEstimate,
                              AltitudeController::controlSignal, measurement,
                              getDroneConfiguration());
    }
}

AltitudeControlSignal
AltitudeController::updateControlSignal(AltitudeReference reference) {

    if (*NEW_HEIGHT_MEASUREMENT_FLAG == 1) {

        // Calculate u_k (unclamped)
        updateControlSignalCodegen(AltitudeController::stateEstimate, reference,
                                   AltitudeController::controlSignal,
                                   AltitudeController::integralWindup,
                                   getDroneConfiguration());

        // Clamp u_k
        clampAltitudeControllerOutput(AltitudeController::controlSignal,
                                      AltitudeController::integralWindup);
    }

    return AltitudeController::controlSignal;
}

void AltitudeController::clampAltitudeControllerOutput(
    AltitudeControlSignal controlSignal,
    AltitudeIntegralWindup integralWindup) {

    real_t ut = AltitudeController::controlSignal.ut;
    if (ut > AltitudeController::utClamp)
        ut = AltitudeController::utClamp;
    else if (ut < -AltitudeController::utClamp)
        ut = -AltitudeController::utClamp;
    AltitudeController::controlSignal.ut = ut;
}

void AltitudeController::initializeController(
    Quaternion quaternion, AltitudeMeasurement altitudeMeasurement,
    AltitudeReference reference) {
    // From now on, attempt to stay at the height we were at on altitude switch
    // (initialized only when going from manual to altitude)

    real_t correctedHeight =
        getCorrectedHeight(altitudeMeasurement.z, quaternion);

    reference.z = altitudeMeasurement.z;
    if (reference.z < AltitudeController::zMin)
        reference.z = AltitudeController::zMin;
    if (reference.z > AltitudeController::zMin)
        reference.z = AltitudeController::zMin;

    AltitudeController::stateEstimate   = {};
    AltitudeController::stateEstimate.z = altitudeMeasurement.z;
    AltitudeController::controlSignal   = {};
    AltitudeController::integralWindup  = {};

    //TODO: rest nog aanpassen
    // Reset final descent timer
    //land_finalDescentCounter = 0;

    // INITIALIZE ALT_THRUST SO THAT WE DON'T START FALLING WHILE WAITING FOR FIRST SONAR MEASUREMENT
    //thrust_out();
}

void AltitudeController::updateReference(AltitudeReference reference) {

    //TODO: getMaxRCThrottle()
    real_t thrust             = getRCThrottle();
    real_t maxSpeedRCThrottle = getMaxSpeedRCThrottle();  // in cm/s

    //TODO: implement sonar functions
    real_t sonarFrequency = getSonarFrequency();

    real_t target_increase;
    if (thrust > AltitudeController::RCThrottleReferenceIncreaseTreshold) {
        target_increase =
            (thrust - AltitudeController::RCThrottleReferenceIncreaseTreshold) *
            maxSpeedRCThrottle / 100.0 / sonarFrequency;
        reference.z += target_increase;
    } else if (thrust <
               AltitudeController::RCThrottleReferenceDecreaseTreshold) {
        target_increase =
            (thrust - AltitudeController::RCThrottleReferenceDecreaseTreshold) *
            maxSpeedRCThrottle / 100.0 / sonarFrequency;
        reference.z += target_increase;
    }

    if (reference.z < AltitudeController::zMin) {
        reference.z = AltitudeController::zMin;
    }

    if (reference.z > AltitudeController::zMax) {
        reference.z = AltitudeController::zMax;
    }
}