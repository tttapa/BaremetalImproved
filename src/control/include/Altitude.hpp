#include <real_t.h>
#include <Quaternion.hpp>

/**
 * Altitude height reference to track, consisting of a single float.
 */
struct AltitudeReference {
    real_t z;  // Height (m)
};

/**
 * Measurement from the sonar, consisting of one float representing the
 * corrected height of the drone, measured in meters.
 */
struct AltitudeMeasurement {
    real_t z;  // Height (m)
};

/**
 * Estimate of the state of the drone's altitude, consisting three components.
 * First is a float representing the marginal angular velocity of the "common
 * motor", relative to the hovering angular velocity. This value is measured
 * in rad/s. Next is a float representing the corrected height of the drone,
 * measured in meters. Finally is a float representing the vertical velocity
 * of the drone, measured in m/s.
 */
struct AltitudeState {
    real_t nt;  // Common motor marginal angular velocity (rad/s)
    real_t z;   // Height (m)
    real_t vz;  // Velocity (m/s)
};

/**
 * Integral of the error of the corrected height of the drone.
 */
struct AltitudeIntegralWindup {
    real_t z;
};

/**
 * PWM control signal sent to the common motor.
 */
struct AltitudeControlSignal {
    real_t ut;  // Common motor marginal signal (/)
};

/**
 * Class to control the altitude of the drone. The first part is an observer to
 * estimate the drone's "common motor" marginal angular velocity, corrected
 * height, and vertical velocity. Next, there is a controller to send the
 * appropriate PWM signal to the common motor based on how far the drone's state
 * estimate deviates from the reference state.
 *
 * To achieve this, the AltitudeController contains variables to store the
 * reference height, state estimate, integral windup and control signal.
 */
class AltitudeController {

  private:
    /**
     * Altitude height reference to track, consisting of a single float.
     */
    AltitudeReference reference;

    /**
     * Measurement from the sonar, consisting of one float representing the
     * corrected height of the drone, measured in meters.
     */
    AltitudeMeasurement measurement;

    /**
     * Estimate of the state of the drone's altitude, consisting three components.
     * First is a float representing the marginal angular velocity of the "common
     * motor", relative to the hovering angular velocity. This value is measured
     * in rad/s. Next is a float representing the corrected height of the drone,
     * measured in meters. Finally is a float representing the vertical velocity
     * of the drone, measured in m/s.
     */
    AltitudeState stateEstimate;

    /**
     * Integral of the error of the corrected height of the drone.
     */
    AltitudeIntegralWindup integralWindup;

    /**
     * PWM control signal sent to the common motor.
     */
    AltitudeControlSignal controlSignal;

    /**
     * Boolean flag indicating whether there is a new measurement from the sonar
     * during the current cycle (238 Hz).
     */
    bool hasNewMeasurement;

    void updateAltitudeKFEstimate(AltitudeState, AltitudeControlSignal,
                                  AltitudeMeasurement, int);

    void getAltitudeControllerOutput(AltitudeState, AltitudeReference,
                                     AltitudeControlSignal,
                                     AltitudeIntegralWindup, int, real_t);

    // TODO: clamp where?
    void clampAltitudeControllerOutput(AltitudeControlSignal,
                                       AltitudeIntegralWindup);

    // TODO: has new measurement - gets hasNewMeasurement
    // void hasNewMeasurement();

    real_t utClamp;
    real_t zMin;
    real_t zMax;
    real_t RCThrottleReferenceIncreaseTreshold;
    real_t RCThrottleReferenceDecreaseTreshold;

  public:
    /**
     * Check if the sonar has sent a new measurement. The observer and controller
     * will only update this cycle (238 Hz) if there is a new measurement. Therefore
     * this function should be called before trying to update the observer or the
     * controller.
     */
    void checkForNewMeasurement();

    void resetNewMeasurementFlag();

    /**
     * Try updating the altitude observer (called at 238 Hz). This function will only
     * change the altitude estimate if there is a new measurement from the sonar. See
     * AltitudeController::checkForNewMeasurement().
     */
    void updateObserver();

    /**
     * Try updating the altitude controller at 238 Hz. This function will only change
     * the altitude control signal if there is a new measurement from the sonar. See
     * AltitudeController::checkForNewMeasurement().
     *
     * @return the control signal to be sent to the "common motor".
     */
    AltitudeControlSignal updateControlSignal();

    void initializeController(Quaternion);

    void updateReference();

};
