#pragma once

/* Includes from src. */
#include <Quaternion.hpp>
#include <real_t.h>

/**
 * Altitude reference height to track, consisting of a single float.
 */
struct AltitudeReference {
    AltitudeReference(real_t z) : z{z} {}
    AltitudeReference() = default;
    real_t z;  ///< Height (m).
};

/**
 * Corrected measurement from the sonar, consisting of one float representing
 * the height of the drone, measured in meters.
 */
struct AltitudeMeasurement {
    AltitudeMeasurement(real_t z) : z{z} {}
    AltitudeMeasurement() = default;
    real_t z;  ///< Height (m).
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
    AltitudeState(real_t nt, real_t z, real_t vz) : nt{nt}, z{z}, vz{vz} {}
    AltitudeState() = default;
    real_t nt;  ///< Common motor marginal angular velocity (rad/s).
    real_t z;   ///< Height (m).
    real_t vz;  ///< Velocity (m/s).
};

/**
 * Integral of the error of the height of the drone.
 */
struct AltitudeIntegralWindup {
    AltitudeIntegralWindup(real_t z) : z{z} {}
    AltitudeIntegralWindup() = default;
    real_t z;  ///< Height (m).
};

/**
 * Marginal PWM control signal sent to the common motor.
 */
struct AltitudeControlSignal {
    AltitudeControlSignal(real_t ut) : ut{ut} {}
    AltitudeControlSignal() = default;
    real_t ut;  ///< Common motor marginal signal (/).
};

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
    /** Marginal PWM control signal sent to the "common motor". */
    AltitudeControlSignal controlSignal;

    /** Integral of the error of the height of the drone. */
    AltitudeIntegralWindup integralWindup;

    /** Corrected height measurement from the sonar. */
    AltitudeMeasurement measurement;

    /** Reference height to track in meters. */
    AltitudeReference reference;

    /**
     * Estimate of the state of the drone's altitude, consisting three
     * components. First is a float representing the marginal angular velocity
     * of the "common motor", relative to the hovering angular velocity. This
     * value is measured in rad/s. Next is a float representing the height of
     * the drone, measured in meters. Finally is a float representing the
     * vertical velocity of the drone, measured in m/s.
     */
    AltitudeState stateEstimate;

  public:
    /**
     * Clamp the current altitude control signal in [-0.10,+0.10].
     */
    void clampControlSignal();

    /**
     * Calculate the current altitude control signal using the code generator.
     * 
     * @param   stateEstimate
     *          Estimate of the current state, determined last cycle.
     * @param   reference
     *          Reference height to track.
     * @param   integralWindup
     *          Current integral windup.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The marginal control signal to be sent to the "common motor"
     *          until the next sonar measurement.
     */
    static AltitudeControlSignal codegenControlSignal(
        AltitudeState stateEstimate, AltitudeReference reference,
        AltitudeIntegralWindup integralWindup, int droneConfiguration);

    /**
     * Calculate the current integral windup using the code generator.
     * 
     * @param   lastIntegralWindup
     *          Integral windup from the last cycle.
     * @param   reference
     *          Reference height to track.
     * 
     * @return  The current integral windup.
     */
    static AltitudeIntegralWindup
    codegenIntegralWindup(AltitudeIntegralWindup integralWindup,
                          AltitudeReference reference,
                          AltitudeState stateEstimate, int droneConfiguration);

    /**
     * Calculate the next altitude estimate using the code generator. Because
     * the altitude control system is implemented with a Kalman filter, this
     * function should be called after AltitudeController::
     * codegenControlSignal() is called in order to determine the state
     * estimate for the next cycle.
     * 
     * @param   stateEstimate
     *          Estimate of the current state, determined last cycle.
     * @param   controlSignal
     *          Marignal control signal that will be sent to the "common motor"
     *          until the next sonar measurement.
     * @param   measurement
     *          Current measurement from the sonar.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The estimate of the next altitude state.
     */
    static AltitudeState codegenNextStateEstimate(
        AltitudeState stateEstimate, AltitudeControlSignal controlSignal,
        AltitudeMeasurement measurement, int droneConfiguration);

    /** Get the altitude controller's control signal. */
    AltitudeControlSignal getControlSignal() { return this->controlSignal; }

    /** Get the altitude controller's integral windup. */
    AltitudeIntegralWindup getIntegralWindup() { return this->integralWindup; }

    /** Get the altitude controller's measurement. */
    AltitudeMeasurement getMeasurement() { return this->measurement; }

    /** Get the altitude controller's reference. */
    AltitudeReference getReference() { return this->reference; }

    /** Get the altitude controller's reference height. */
    real_t getReferenceHeight() { return this->reference.z; }

    /** Get the altitude controller's state estimate. */
    AltitudeState getStateEstimate() { return this->stateEstimate; }

    /**
     * Reset the altitude controller. Set the estimate height and the reference
     * height to the given height.
     */
    void init(real_t correctedSonarMeasurement);

    /**
     * Set the altitude controller's reference height.
     * 
     * @param   reference
     *          New reference height to track in meters.
     */
    void setReference(AltitudeReference reference);

    /**
     * Update the altitude controller with the altitude controller's reference
     * height. Use updateRCReference() to update the reference heigth based on
     * the RC throttle and setReference() to set it directly.  This function
     * should only be called when there is a new measurement from the sonar.
     * 
     * @return  The marginal control signal to be sent to the "common motor"
     *          until the next sonar measurement.
     */
    AltitudeControlSignal updateControlSignal();

    /**
     * Update the altitude observer with the given measurement height. This
     * function should only be called when there is a new measurement from the
     * sonar. Because the altitude control system is implemented with a Kalman
     * filter, this function should be called after AltitudeController::
     * updateControlSignal() is called in order to determine the state estimate
     * for the next cycle.
     * 
     * @param   measurement
     *          New height measurement from the sonar.
     */
    void updateObserver(AltitudeMeasurement measurement);

    /**
     * Update the altitude controller's reference height using the RC throttle.
     * When the RC throttle is in the dead zone [25%, 75%], the reference height
     * will not change. If the value of the RC throttle exceeds 75% (goes below
     * 25%), then the reference height will increase (decrease). The maximum
     * increase (decrease) speed is reached when the value of the RC throttle
     * reaches 100% (0%).
     */
    void updateRCReference();
};
