#pragma once
#include <Autonomous.hpp>     ///< AutonomousState
#include <LoggerStructs.hpp>  ///< FlightMode
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
class BiasManager {

  private:
    /**
     * Hovering thrust that should be used by the autonomous controller as a
     * base value for the actual hovering thrust. This is set whenever the pilot
     * leaves ALTITUDE-HOLD mode.
     */
    real_t autonomousHoveringThrust = 0.0;

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
     * Get the hovering thrust that should be used by the autonomous controller
     * as a base value for the actual hovering thrust. This is set whenever the
     * pilot leaves ALTITUDE-HOLD mode.
     */
    real_t getAutonomousHoveringThrust() { return autonomousHoveringThrust; }

    /** Get the current bias to be added to the RC roll. */
    real_t getRollBias() { return rollBias; }

    /** Get the current bias to be added to the RC pitch. */
    real_t getPitchBias() { return pitchBias; }

    /**
     * Get the current bias to be added to the altitude controller's
     * marginal thrust.
     */
    real_t getThrustBias() { return thrustBias; }

    /**
     * Set the hovering thrust to the given value. This should be called when
     * switching from ALTITUDE-HOLD mode to either MANUAL or AUTONOMOUS mode.
     * This value will be used as a base hovering thrust by the AUTONOMOUS
     * controller. This also means that the pilot must fly in ALTITUDE-HOLD mode
     * before initiating AUTONOMOUS mode from the ground.
     * 
     * This function only has effect if the given hovering thrust is greater
     * than or equal to 0.30.
     * 
     * @param   hoveringThrust
     *          New hovering thrust.
     */
    void setAutonomousHoveringThrust(real_t hoveringThrust);

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
     * @param   flightMode
     *          The flight mode of the drone (MANUAL/ALTITUTDE_HOLD/AUTONOMOUS).
     */
    void updateThrustBias(real_t commonThrust, FlightMode flightMode);
};