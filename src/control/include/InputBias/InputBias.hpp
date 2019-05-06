#pragma once
#include <Autonomous.hpp>
#include <Globals.hpp>
#include <real_t.h>

/**
 * Class to keep track of and to update the drone's three input biases. The
 * first two are biases to add to the RC roll and pitch. In steady state, these
 * biases should cancel out the inevitable orientation drift and the pilot
 * should be able to fly straight with almost zero roll/pitch action. The third
 * bias is used when the altitude controller is active: it is added to the 
 * altitude controller's marginal thrust signal to get the signal to be sent to
 * the "common motor". In steady state, this will converge to the hovering
 * thrust.
 */
class InputBias {

  private:
    /**
     * Bias to be added to the RC roll. In steady state, this will evolve to
     * the equilibrium roll. The unit of this variable is radians.
     */
    real_t rollBias;

    /**
     * Bias to be added to the RC pitch. In steady state, this will evolve
     * to the equilibrium pitch. The unit of this variable is radians.
     */
    real_t pitchBias;

    /**
     * Bias to be added to the altitude controller's marginal thrust. In
     * steady state, this will evolve to the hovering thrust. This variable has
     * no unit (duty cycle of "common motor" PWM signal), and must always be in
     * [0,1].
     */
    real_t thrustBias;

    /**
     * Weight used in the exponential filters for the roll and pitch biases when
     * the pilot is in control of these parameters, i.e. the MANUAL and
     * ALTITUDE_HOLD flight modes.
     */
    const real_t ROTATION_BIAS_WEIGHT_PILOT = 0.001;

    /**
     * Weight used in the exponential filters for the roll and pitch biases when
     * the drone is loitering in the AUTONOMOUS flight mode.
     */
    const real_t ROTATION_BIAS_WEIGHT_LOITERING = 0.001;

    /**
     * Weight used in the exponential filters for the roll and pitch biases when
     * the drone is navigating in the AUTONOMOUS flight mode.
     */
    const real_t ROTATION_BIAS_WEIGHT_NAVIGATING = 0.00005;

    /**
     * Weight used in the exponential filters for the thrust bias when the drone
     * is flying in the MANUAL flight mode.
     */
    const real_t THRUST_BIAS_WEIGHT_MANUAL = 0.01;

    /**
     * Weight used in the exponential filters for the thrust bias when the drone
     * is flying in the ALTITUDE_HOLD flight mode or is holding its altitude in
     * the AUTONOMOUS flight mode.
     */
    // TODO: check if this is ok. should be ~10-30s of bad bias before it fixes
    const real_t THRUST_BIAS_WEIGHT_ALTITUDE_HOLD = 0.0001;

  public:
    /**
     * Get the current bias to be added to the RC roll.
     */
    real_t getRollBias() { return rollBias; }

    /**
     * Get the current bias to be added to the RC pitch.
     */
    real_t getPitchBias() { return pitchBias; }

    /**
     * Get the current bias to be added to the altitude controller's
     * marginal thrust.
     */
    real_t getThrustBias() { return thrustBias; }

    /**
     * Update the roll bias (exponential filter) using the given RC roll in
     * radians.
     * 
     * @param   referenceRollRads
     *          Reference roll to be sent to the attitude controller in radians.
     * @param   flightMode
     *          The flight mode of the drone (MANUAL/ALTITUTDE_HOLD/AUTONOMOUS).
     */
    void updateRollBias(real_t referenceRollRads, FlightMode flightMode);

    /**
     * Update the roll bias (exponential filter) using the given RC roll in
     * radians.
     * 
     * @param   referenceRollRads
     *          Reference roll to be sent to the attitude controller in radians.
     * @param   flightMode
     *          The flight mode of the drone (MANUAL/ALTITUTDE_HOLD/AUTONOMOUS).
     * @param   autonomousState
     *          The state the autonomous controller is in, which should only be
     *          specified if the flight mode is AUTONOMOUS.
     */
    void updateRollBias(real_t referenceRollRads, FlightMode flightMode,
                        AutonomousState autonomousState);

    /**
     * Update the pitch bias (exponential filter) using the given RC pitch in
     * radians.
     * 
     * @param   referencePitchRads
     *          Reference pitch to be sent to the attitude controller in
     *          radians.
     * @param   flightMode
     *          The flight mode of the drone (MANUAL/ALTITUTDE_HOLD/AUTONOMOUS).
     */
    void updatePitchBias(real_t referencePitchRads, FlightMode flightMode);

    /**
     * Update the pitch bias (exponential filter) using the given RC pitch in
     * radians.
     * 
     * @param   referencePitchRads
     *          Reference pitch to be sent to the attitude controller in
     *          radians.
     * @param   flightMode
     *          The flight mode of the drone (MANUAL/ALTITUTDE_HOLD/AUTONOMOUS).
     * @param   autonomousState
     *          The state the autonomous controller is in, which should only be
     *          specified if the flight mode is AUTONOMOUS.
     */
    void updatePitchBias(real_t referencePitchRads, FlightMode flightMode,
                         AutonomousState autonomousState);

    /**
     * Update the thrust bias (exponential filter) using the given RC thrust.
     * 
     * @param   rcThrust
     *          Thrust sent by the RC in [0,1].
     */
    void updateThrustBiasManual(real_t rcThrust);

    /**
     * Update the thrust bias (exponential filter) using the marginal thrust
     * signal from the altitude controller.
     * 
     * @param   ut
     *          Marginal thrust sent by the altitude controller.
     */
    void updateThrustBiasAltitudeHold(real_t ut);
};