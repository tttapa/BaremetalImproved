#pragma once
#include <Autonomous.hpp>
#include <RCValues.hpp>
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
    real_t rollBias = 0.0;

    /**
     * Bias to be added to the RC pitch. In steady state, this will evolve
     * to the equilibrium pitch. The unit of this variable is radians.
     */
    real_t pitchBias = 0.0;

    /**
     * Bias to be added to the altitude controller's marginal thrust. In
     * steady state, this will evolve to the hovering thrust. This variable has
     * no unit (duty cycle of "common motor" PWM signal), and must always be in
     * [0,1].
     */
    real_t thrustBias = 0.0;

  public:
    /**
     * Reset the input bias.
     */
    void init();

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
     * Update the thrust bias (exponential filter) using the given common
     * thrust.
     * 
     * @param   commonThrust
     *          Signal sent to the common motor in [0,1].
     */
    void updateThrustBias(real_t commonThrust);

};