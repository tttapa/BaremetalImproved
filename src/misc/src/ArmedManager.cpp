#include <ArmedManager.hpp>

/* Includes from src. */
#include <MiscInstances.hpp>
#include <RCValues.hpp>
#include <Time.hpp>

#pragma region Constants
/** The armed status can only change if the throttle stays below 0.03. */
static constexpr float THROTTLE_THRESHOLD = 0.03;

/** The drone can only be disarmed if the yaw says below -0.48. */
static constexpr float YAW_LOWER_THRESHOLD = -0.48;

/** The drone can only be armed if the yaw says above 0.48. */
static constexpr float YAW_UPPER_THRESHOLD = 0.48;

/**
 * The pilot must hold the left joystick in the lower-right or -left corner for
 * 2 seconds in order to change the armed status.
 */
static constexpr float ARMED_CHANGE_DELAY = 2.0;
#pragma endregion

void ArmedManager::update() {

    float throttle = getThrottle();
    float yaw      = -getYaw();

    /* Only do arming/disarming if we're in the changing zone. */
    if (throttle > THROTTLE_THRESHOLD ||
        (yaw > YAW_LOWER_THRESHOLD && yaw < YAW_UPPER_THRESHOLD)) {
        this->isWaitingForChange = false;
        return;
    }

    /* Only do arming if the buzzer is not busy. */
    if (buzzerManager.isInstructionBusy())
        return;

    /* If we're waiting for the armed status to change. */
    if (this->isWaitingForChange) {

        /* Change armed status if we've waited long enough. */
        if (getTime() - waitingStartTime >= ARMED_CHANGE_DELAY) {
            this->isWaitingForChange = false;
            if (yaw >= YAW_UPPER_THRESHOLD) {
                this->armed = true;
                buzzerManager.addArmedBeeps();
            } else {
                this->armed = false;
                buzzerManager.addDisarmedBeeps();
            }
        }
    } else {

        /* Record the time at which we started waited. */
        this->isWaitingForChange = true;
        waitingStartTime         = getTime();
    }
}