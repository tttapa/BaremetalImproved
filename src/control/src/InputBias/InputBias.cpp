#include <BaremetalCommunicationDef.hpp>
#include <InputBias/InputBias.hpp>

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
