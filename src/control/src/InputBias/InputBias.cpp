#include <InputBias/InputBias.hpp>

void InputBias::updateRollBias(real_t referenceRollRads,
                               FlightMode flightMode) {
    InputBias::updateRollBias(referenceRollRads, flightMode, IDLE_GROUND);
}

void InputBias::updateRollBias(real_t referenceRollRads, FlightMode flightMode,
                               AutonomousState autonomousState) {
    if (flightMode == MANUAL_MODE || flightMode == ALTITUDE_HOLD_MODE) {
        this->rollBias += ROTATION_BIAS_WEIGHT_PILOT * (rcRollRads)
    }

    else if (flightMode == FlightMode::AUTONOMOUS_MODE) {

        if (autonomousState == AutonomousState::LOITERING) {
            this->rollBias +=
                ROTATION_BIAS_WEIGHT_LOITERING * (rcRollRads - this->rollBias);
        } else {
            this->rollBias +=
                ROTATION_BIAS_WEIGHT_NAVIGATING * (rcRollRads - this->rollBias);
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
    if (flightMode == MANUAL_MODE || flightMode == ALTITUDE_HOLD_MODE) {
        this->pitchBias += ROTATION_BIAS_WEIGHT_PILOT * (rcPitchRads)
    }

    else if (flightMode == FlightMode::AUTONOMOUS_MODE) {

        if (autonomousState == AutonomousState::LOITERING) {
            this->pitchBias += ROTATION_BIAS_WEIGHT_LOITERING *
                               (rcPitchRads - this->pitchBias);
        } else {
            this->pitchBias += ROTATION_BIAS_WEIGHT_NAVIGATING *
                               (rcPitchRads - this->pitchBias);
        }
    }
}

void InputBias::updateThrustBiasManual(real_t rcThrust) {
    this->thrustBias +=
        THRUST_BIAS_WEIGHT_MANUAL * (rcThrust - this->thrustBias);
}

void InputBias::updateThrustBiasAltitudeHold(real_t ut) {
    this->thrustBias += THRUST_BIAS_WEIGHT_ALTITUDE_HOLD * ut;
}
