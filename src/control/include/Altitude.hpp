#pragma once
#include <Quaternion.hpp>
#include <real_t.h>

/**
 * Altitude reference height to track, consisting of a single float.
 */
struct AltitudeReference {
    real_t z; /* Height (m) */
};

/**
 * Corrected measurement from the sonar, consisting of one float representing
 * the height of the drone, measured in meters.
 */
struct AltitudeMeasurement {
    real_t z; /* Height (m) */
};

/**
 * Estimate of the state of the drone's altitude, consisting three components.
 * First is a float representing the marginal angular velocity of the "common
 * motor", relative to the hovering angular velocity. This value is measured
 * in rad/s. Next is a float representing the height of the drone, measured
 * in meters. Finally is a float representing the vertical velocity of the
 * drone, measured in m/s.
 */
struct AltitudeState {
    real_t nt; /* Common motor marginal angular velocity (rad/s) */
    real_t z;  /* Height (m) */
    real_t vz; /* Velocity (m/s) */
};

/**
 * Integral of the error of the height of the drone.
 */
struct AltitudeIntegralWindup {
    real_t z; /* Height (m) */
};

/**
 * Marginal PWM control signal sent to the common motor.
 */
struct AltitudeControlSignal {
    real_t ut; /* Common motor marginal signal (/) */
};

/**
 * Update the reference height using the RC throttle. When the RC throttle is in
 * the dead zone [25%, 75%], the reference height will not change. If the value
 * of the RC throttle exceeds 75% (goes below 25%), then the reference height
 * will increase (decrease). The maximum increase (decrease) speed is reached
 * when the value of the RC throttle reaches 100% (0%).
 * 
 * @param   reference
 *          reference height from the previous cycle
 * 
 * @return  the updated reference height.
 */
AltitudeReference rcUpdateReferenceHeight(AltitudeReference reference);

/**
 * Class to control the altitude of the drone. The first part is an observer to
 * estimate the drone's "common motor" marginal angular velocity, height and
 * vertical velocity. Next, there is a controller to send the appropriate
 * marginal PWM signal to the common motor based on how far the drone's state
 * estimate deviates from the reference state.
 *
 * To achieve this, the AltitudeController contains variables to store the
 * state estimate, integral windup, and control signal.
 */
class AltitudeController {

  private:
    /**
     * Estimate of the state of the drone's altitude, consisting three
     * components. First is a float representing the marginal angular velocity
     * of the "common motor", relative to the hovering angular velocity. This
     * value is measured in rad/s. Next is a float representing the height of
     * the drone, measured in meters. Finally is a float representing the
     * vertical velocity of the drone, measured in m/s.
     */
    AltitudeState stateEstimate;

    /**
     * Integral of the error of the height of the drone.
     */
    AltitudeIntegralWindup integralWindup;

    /**
     * Marginal PWM control signal sent to the "common motor".
     */
    AltitudeControlSignal controlSignal;

    /**
     * Calculate the current altitude control signal using the code generator.
     * 
     * @param   stateEstimate
     *          estimate of the current state, determined last cycle
     * @param   reference
     *          reference height to track
     * @param   integralWindup
     *          current integral windup
     * @param   droneConfiguration
     *          configuration of the drone
     * 
     * @return  the marginal control signal to be sent to the "common motor"
     *          until the next sonar measurement.
     */
    AltitudeControlSignal codegenControlSignal(
        AltitudeState stateEstimate, AltitudeReference reference,
        AltitudeIntegralWindup integralWindup, int droneConfiguration);

    /**
     * Calculate the current integral windup using the code generator.
     * 
     * @param   lastIntegralWindup
     *          integral windup from the last cycle
     * @param   reference
     *          reference height to track
     * 
     * @return  the current integral windup.
     */
    AltitudeIntegralWindup
    codegenIntegralWindup(AltitudeIntegralWindup lastIntegralWindup,
                          AltitudeReference reference);

    /**
     * Calculate the next altitude estimate using the code generator. Because
     * the altitude control system is implemented with a Kalman filter, this
     * function should be called after AltitudeController::
     * codegenControlSignal() is called in order to determine the state
     * estimate for the next cycle.
     * 
     * @param   stateEstimate
     *          estimate of the current state, determined last cycle
     * @param   controlSignal
     *          marignal control signal that will be sent to the "common motor"
     *          until the next sonar measurement
     * @param   measurement
     *          current measurement from the sonar
     * @param   droneConfiguration
     *          configuration of the drone
     * 
     * @return  the estimate of the next altitude state.
     */
    AltitudeState codegenNextStateEstimate(AltitudeState stateEstimate,
                                           AltitudeControlSignal controlSignal,
                                           AltitudeMeasurement measurement,
                                           int droneConfiguration);

    /**
     * Clamp the given altitude control signal in [-0.10,+0.10].
     * 
     * @param   controlSignal
     *          control signal to clamp
     * 
     * @return  the clamped altitude control signal.
     */
    AltitudeControlSignal
    clampControlSignal(AltitudeControlSignal controlSignal);

  public:
    /**
     * Update the altitude observer with the given measurement height. This
     * function should only be called when there is a new measurement from the
     * sonar. Because the altitude control system is implemented with a Kalman
     * filter, this function should be called after AltitudeController::
     * updateControlSignal() is called in order to determine the state estimate
     * for the next cycle.
     * 
     * @param   measurement
     *          new height measurement from the sonar
     */
    void updateObserver(AltitudeMeasurement measurement);

    /**
     * Update the altitude controller with the given reference height. This
     * function should only be called when there is a new measurement from the
     * sonar.
     * 
     * @param   reference
     *          the reference height to track
     *
     * @return  the marginal control signal to be sent to the "common motor"
     *          until the next sonar measurement.
     */
    AltitudeControlSignal updateControlSignal(AltitudeReference reference);

    /**
     * Reset the altitude controller.
     */
    void init();
};
