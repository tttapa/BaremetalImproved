#include <InputBias.hpp>

void InputBias::updateRollBias(real_t referenceRollRads,
                               FlightMode flightMode) {
    InputBias::updateRollBias(referenceRollRads, flightMode, 0);
}

void InputBias::updateRollBias(real_t referenceRollRads, FlightMode flightMode,
                               AutonomousState autonomousState) {
    if (flightMode == MANUAL_MODE || flightMode == ALTITUDE_HOLD_MODE) {
        InputBias::rollBias += ROTATION_BIAS_WEIGHT_PILOT * (rcRollRads)
    }

    else if (flightMode == FlightMode::AUTONOMOUS_MODE) {

        if (autonomousState == AutonomousState::LOITERING) {
            InputBias::rollBias += ROTATION_BIAS_WEIGHT_LOITERING *
                                   (rcRollRads - InputBias::rollBias);
        } else {
            InputBias::rollBias += ROTATION_BIAS_WEIGHT_NAVIGATING *
                                   (rcRollRads - InputBias::rollBias);
        }
    }
}

void InputBias::updatePitchBias(real_t referencePitchRads,
                                FlightMode flightMode) {
    InputBias::updatePitchBias(referencePitchRads, flightMode, 0);
}

void InputBias::updatePitchBias(real_t referencePitchRads,
                                FlightMode flightMode,
                                AutonomousState autonomousState) {
    if (flightMode == MANUAL_MODE || flightMode == ALTITUDE_HOLD_MODE) {
        InputBias::pitchBias += ROTATION_BIAS_WEIGHT_PILOT * (rcPitchRads)
    }

    else if (flightMode == FlightMode::AUTONOMOUS_MODE) {

        if (autonomousState == AutonomousState::LOITERING) {
            InputBias::pitchBias += ROTATION_BIAS_WEIGHT_LOITERING *
                                    (rcPitchRads - InputBias::pitchBias);
        } else {
            InputBias::pitchBias += ROTATION_BIAS_WEIGHT_NAVIGATING *
                                    (rcPitchRads - InputBias::pitchBias);
        }
    }
}

void InputBias::updateThrustBiasManual(real_t rcThrust) {
    InputBias::thrustBias +=
        THRUST_BIAS_WEIGHT_MANUAL * (rcThrust - InputBias::thrustBias);
}

void InputBias::updateThrustBiasAltitudeHold(real_t ut) {
    InputBias::thrustBias += THRUST_BIAS_WEIGHT_ALTITUDE_HOLD * ut;
}
