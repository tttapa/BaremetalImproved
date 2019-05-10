#include <BaremetalCommunicationDef.hpp>  ///< FlightMode
#include <InputBias/InputBias.hpp>

/**
 * Weight used in the exponential filters for the roll and pitch biases when
 * the pilot is in control of these parameters, i.e. the MANUAL and
 * ALTITUDE_HOLD flight modes.
 */
static constexpr real_t ROTATION_BIAS_WEIGHT_PILOT = 0.001;

/**
 * Weight used in the exponential filters for the roll and pitch biases when
 * the drone is loitering in the AUTONOMOUS flight mode.
 */
static constexpr real_t ROTATION_BIAS_WEIGHT_LOITERING = 0.001;

/**
 * Weight used in the exponential filters for the roll and pitch biases when
 * the drone is navigating in the AUTONOMOUS flight mode.
 */
static constexpr real_t ROTATION_BIAS_WEIGHT_NAVIGATING = 0.00005;

/**
 * Weight used in the exponential filters for the thrust bias when the drone
 * is flying in the MANUAL flight mode.
 */
static constexpr real_t THRUST_BIAS_WEIGHT_MANUAL = 0.01;

/**
 * Weight used in the exponential filters for the thrust bias when the drone
 * is flying in the ALTITUDE_HOLD flight mode or is holding its altitude in
 * the AUTONOMOUS flight mode.
 */
// TODO: check if this is ok. should be ~10-30s of bad bias before it fixes
static constexpr real_t THRUST_BIAS_WEIGHT_ALTITUDE_HOLD = 0.0001;

void InputBias::init() {
    this->pitchBias  = 0.0;
    this->rollBias   = 0.0;
    this->thrustBias = 0.0;
}

void InputBias::updateRollBias(real_t referenceRollRads,
                               FlightMode flightMode) {
    InputBias::updateRollBias(referenceRollRads, flightMode, IDLE_GROUND);
}

void InputBias::updateRollBias(real_t referenceRollRads, FlightMode flightMode,
                               AutonomousState autonomousState) {
    if (flightMode == FlightMode::MANUAL ||
        flightMode == FlightMode::ALTITUDE_HOLD) {
        this->rollBias += ROTATION_BIAS_WEIGHT_PILOT * (referenceRollRads);
    }

    else if (flightMode == FlightMode::AUTONOMOUS) {

        if (autonomousState == AutonomousState::LOITERING) {
            this->rollBias += ROTATION_BIAS_WEIGHT_LOITERING *
                              (referenceRollRads - this->rollBias);
        } else {
            this->rollBias += ROTATION_BIAS_WEIGHT_NAVIGATING *
                              (referenceRollRads - this->rollBias);
        }
    }
}

void InputBias::updatePitchBias(real_t referencePitchRads,
                                FlightMode flightMode) {
    InputBias::updatePitchBias(referencePitchRads, flightMode, IDLE_GROUND);
}

void InputBias::updatePitchBias(real_t referencePitchRads,
                                FlightMode flightMode,
                                AutonomousState autonomousState) {
    if (flightMode == FlightMode::MANUAL ||
        flightMode == FlightMode::ALTITUDE_HOLD) {
        this->pitchBias += ROTATION_BIAS_WEIGHT_PILOT * (referencePitchRads);
    }

    else if (flightMode == FlightMode::AUTONOMOUS) {

        if (autonomousState == AutonomousState::LOITERING) {
            this->pitchBias += ROTATION_BIAS_WEIGHT_LOITERING *
                               (referencePitchRads - this->pitchBias);
        } else {
            this->pitchBias += ROTATION_BIAS_WEIGHT_NAVIGATING *
                               (referencePitchRads - this->pitchBias);
        }
    }
}

void InputBias::updateThrustBias(real_t commonThrust) {
    this->thrustBias +=
        THRUST_BIAS_WEIGHT_MANUAL * (commonThrust - this->thrustBias);
}
