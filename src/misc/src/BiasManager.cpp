#include <BiasManager.hpp>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>

#pragma region Constants
/**
 * If the main interrupt tries to set the autonomous controller's base hovering
 * thrust equal to any value less than 0.30, it will be ignored. This base
 * value is used for taking off in autonomous mode.
 */
static constexpr float AUTONOMOUS_HOVERING_THRUST_THRESHOLD = 0.30;

/**
 * Weight used in the exponential filters for the roll and pitch biases when
 * the pilot is in control of these parameters, i.e. the MANUAL and
 * ALTITUDE_HOLD flight modes. This base weight is for an IMU frequency of 119
 * Hz and will be adjusted for the actual IMU frequency.
 */
static constexpr float ROTATION_BIAS_WEIGHT_PILOT = 0.005;

/**
 * Weight used in the exponential filters for the roll and pitch biases when
 * the drone is loitering in the AUTONOMOUS flight mode. This base weight is for
 * an IMU frequency of 119 Hz and will be adjusted for the actual IMU frequency.
 */
// TODO: turn this back on, but lower! Maybe 0.0005
//static constexpr float ROTATION_BIAS_WEIGHT_LOITERING = 0.001;
static constexpr float ROTATION_BIAS_WEIGHT_LOITERING = 0.0f;

/**
 * Weight used in the exponential filters for the roll and pitch biases when
 * the drone is navigating in the AUTONOMOUS flight mode. This base weight is
 * for an IMU frequency of 119 Hz and will be adjusted for the actual IMU
 * frequency.
 */
// TODO: should this be zero or non-zero? I think it should be zero
//static constexpr float ROTATION_BIAS_WEIGHT_NAVIGATING = 0.0002;
static constexpr float ROTATION_BIAS_WEIGHT_NAVIGATING = 0.0f;

/**
 * Weight used in the exponential filters for the thrust bias when the drone
 * is flying in the MANUAL flight mode. This base weight is for an IMU frequency
 * of 119 Hz and will be adjusted for the actual IMU frequency.
 */
static constexpr float THRUST_BIAS_WEIGHT_MANUAL = 0.01;

/**
 * Weight used in the exponential filters for the thrust bias when the drone
 * is flying in the ALTITUDE_HOLD flight mode or is holding its altitude in
 * the AUTONOMOUS flight mode. This base weight is for an IMU frequency of 119
 * Hz and will be adjusted for the actual IMU frequency.
 */
static constexpr float THRUST_BIAS_WEIGHT_ALTITUDE_HOLD = 0.0004;
#pragma endregion

void BiasManager::init() {
    this->pitchBias  = 0.0;
    this->rollBias   = 0.0;
    this->thrustBias = 0.0;
}

void BiasManager::setAutonomousHoveringThrust(float hoveringThrust) {
    if(hoveringThrust >= AUTONOMOUS_HOVERING_THRUST_THRESHOLD)
        this->autonomousHoveringThrust = hoveringThrust;
}

void BiasManager::updateRollBias(float referenceRollRads,
                                 FlightMode flightMode) {
    BiasManager::updateRollBias(referenceRollRads, flightMode, IDLE_GROUND);
}

void BiasManager::updateRollBias(float referenceRollRads,
                                 FlightMode flightMode,
                                 AutonomousState autonomousState) {

    float weight = 0.0;
    if (flightMode == FlightMode::MANUAL || flightMode == FlightMode::ALTITUDE_HOLD) {
        weight = ROTATION_BIAS_WEIGHT_PILOT / IMU_FACTOR;
    } else if (flightMode == FlightMode::AUTONOMOUS) {
        if (autonomousState == AutonomousState::LOITERING)
            weight = ROTATION_BIAS_WEIGHT_LOITERING / IMU_FACTOR;
        else
            weight = ROTATION_BIAS_WEIGHT_NAVIGATING / IMU_FACTOR;
    }

    this->rollBias += weight * (referenceRollRads - this->rollBias);
}

void BiasManager::updatePitchBias(float referencePitchRads,
                                  FlightMode flightMode) {
    BiasManager::updatePitchBias(referencePitchRads, flightMode, IDLE_GROUND);
}

void BiasManager::updatePitchBias(float referencePitchRads,
                                  FlightMode flightMode,
                                  AutonomousState autonomousState) {

    float weight = 0.0;
    if (flightMode == FlightMode::MANUAL || flightMode == FlightMode::ALTITUDE_HOLD) {
        weight = ROTATION_BIAS_WEIGHT_PILOT / IMU_FACTOR;
    } else if (flightMode == FlightMode::AUTONOMOUS) {
        if (autonomousState == AutonomousState::LOITERING)
            weight = ROTATION_BIAS_WEIGHT_LOITERING / IMU_FACTOR;
        else
            weight = ROTATION_BIAS_WEIGHT_NAVIGATING / IMU_FACTOR;
    }

    this->pitchBias += weight * (referencePitchRads - this->pitchBias);
}

void BiasManager::updateThrustBias(float commonThrust, FlightMode flightMode) {

    float weight = 0.0;
    if(flightMode == FlightMode::MANUAL)
        weight = THRUST_BIAS_WEIGHT_MANUAL / IMU_FACTOR;
    else if(flightMode == FlightMode::ALTITUDE_HOLD || flightMode == FlightMode::AUTONOMOUS)
        weight = THRUST_BIAS_WEIGHT_ALTITUDE_HOLD / IMU_FACTOR;

    this->thrustBias += weight * (commonThrust - this->thrustBias);

}
