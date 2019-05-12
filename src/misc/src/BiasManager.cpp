#include <BiasManager.hpp>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>

#pragma region Constants
/**
 * Weight used in the exponential filters for the roll and pitch biases when
 * the pilot is in control of these parameters, i.e. the MANUAL and
 * ALTITUDE_HOLD flight modes. This base weight is for an IMU frequency of 119
 * Hz and will be adjusted for the actual IMU frequency.
 */
static constexpr real_t ROTATION_BIAS_WEIGHT_PILOT = 0.001;

/**
 * Weight used in the exponential filters for the roll and pitch biases when
 * the drone is loitering in the AUTONOMOUS flight mode. This base weight is for
 * an IMU frequency of 119 Hz and will be adjusted for the actual IMU frequency.
 */
static constexpr real_t ROTATION_BIAS_WEIGHT_LOITERING = 0.001;

/**
 * Weight used in the exponential filters for the roll and pitch biases when
 * the drone is navigating in the AUTONOMOUS flight mode. This base weight is
 * for an IMU frequency of 119 Hz and will be adjusted for the actual IMU
 * frequency.
 */
static constexpr real_t ROTATION_BIAS_WEIGHT_NAVIGATING = 0.00005;

/**
 * Weight used in the exponential filters for the thrust bias when the drone
 * is flying in the MANUAL flight mode. This base weight is for an IMU frequency
 * of 119 Hz and will be adjusted for the actual IMU frequency.
 */
static constexpr real_t THRUST_BIAS_WEIGHT_MANUAL = 0.01;

/**
 * Weight used in the exponential filters for the thrust bias when the drone
 * is flying in the ALTITUDE_HOLD flight mode or is holding its altitude in
 * the AUTONOMOUS flight mode. This base weight is for an IMU frequency of 119
 * Hz and will be adjusted for the actual IMU frequency.
 */
static constexpr real_t THRUST_BIAS_WEIGHT_ALTITUDE_HOLD = 0.0001;
#pragma endregion

void BiasManager::init() {
    this->pitchBias  = 0.0;
    this->rollBias   = 0.0;
    this->thrustBias = 0.0;
}

void BiasManager::updateRollBias(real_t referenceRollRads,
                                 FlightMode flightMode) {
    BiasManager::updateRollBias(referenceRollRads, flightMode, IDLE_GROUND);
}

void BiasManager::updateRollBias(real_t referenceRollRads,
                                 FlightMode flightMode,
                                 AutonomousState autonomousState) {

    real_t weight1 = ROTATION_BIAS_WEIGHT_PILOT / IMU_FACTOR;
    real_t weight2 = ROTATION_BIAS_WEIGHT_LOITERING / IMU_FACTOR;
    real_t weight3 = ROTATION_BIAS_WEIGHT_NAVIGATING / IMU_FACTOR;

    if (flightMode == FlightMode::MANUAL ||
        flightMode == FlightMode::ALTITUDE_HOLD) {
        this->rollBias += weight1 * (referenceRollRads);

    } else if (flightMode == FlightMode::AUTONOMOUS) {

        if (autonomousState == AutonomousState::LOITERING) {
            this->rollBias += weight2 * (referenceRollRads - this->rollBias);
        } else {
            this->rollBias += weight3 * (referenceRollRads - this->rollBias);
        }
    }
}

void BiasManager::updatePitchBias(real_t referencePitchRads,
                                  FlightMode flightMode) {
    BiasManager::updatePitchBias(referencePitchRads, flightMode, IDLE_GROUND);
}

void BiasManager::updatePitchBias(real_t referencePitchRads,
                                  FlightMode flightMode,
                                  AutonomousState autonomousState) {

    real_t weight1 = ROTATION_BIAS_WEIGHT_PILOT / IMU_FACTOR;
    real_t weight2 = ROTATION_BIAS_WEIGHT_LOITERING / IMU_FACTOR;
    real_t weight3 = ROTATION_BIAS_WEIGHT_NAVIGATING / IMU_FACTOR;

    if (flightMode == FlightMode::MANUAL ||
        flightMode == FlightMode::ALTITUDE_HOLD) {
        this->pitchBias += weight1 * (referencePitchRads);

    } else if (flightMode == FlightMode::AUTONOMOUS) {

        if (autonomousState == AutonomousState::LOITERING) {
            this->pitchBias += weight2 * (referencePitchRads - this->pitchBias);
        } else {
            this->pitchBias += weight3 * (referencePitchRads - this->pitchBias);
        }
    }
}

void BiasManager::updateThrustBias(real_t commonThrust, FlightMode flightMode) {

    real_t weight1 = THRUST_BIAS_WEIGHT_MANUAL / IMU_FACTOR;
    //real_t weight2 = THRUST_BIAS_WEIGHT_ALTITUDE_HOLD / IMU_FACTOR;

    if (flightMode == FlightMode::MANUAL) {
        this->thrustBias += weight1 * (commonThrust - this->thrustBias);

    } else if (flightMode == FlightMode::ALTITUDE_HOLD ||
               flightMode == FlightMode::AUTONOMOUS) {
        this->thrustBias += 0; //weight2 * (commonThrust - this->thrustBias);
    }
}
