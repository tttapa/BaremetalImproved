#include <ConfigurationManager.hpp>

/* Includes from src. */
#include <MiscInstances.hpp>
#include <RCValues.hpp>
#include <Time.hpp>

#pragma region Constants
/** 3 wiggles to sound configuration. */
static constexpr int NUM_WIGGLES = 3;
/** Every 1000 ms, 1 wiggle decreases. */
static constexpr float WIGGLE_DECREMENT_DELAY = 1.0;

/**
 * The user must be passed the upper/lower threshold for 1.5 seconds before the
 * configuration changes.
 */
static constexpr float CONFIG_CHANGE_DELAY = 1.5;

/**
 * After the configuration has changed, there will be a 0.5 second pause until
 * the next warning.
 */
static constexpr float AFTER_CONFIG_CHANGE_DELAY = 0.5;

/** Threshold for incrementing the cofiguration is 0.80. */
static constexpr float CONFIG_UPPER_THRESHOLD = 0.80;
/** Threshold for decrementing the cofiguration is -0.80. */
static constexpr float CONFIG_LOWER_THRESHOLD = -0.80;
/** Upper threshold for counting wiggles is 0.08. */
static constexpr float WIGGLE_UPPER_THRESHOLD = 0.08;
/** Lower threshold for counting wiggles is -0.08. */
static constexpr float WIGGLE_LOWER_THRESHOLD = -0.08;

/** Configuration can only be changed when the RC throttle is below 0.03. */
static constexpr float THROTTLE_THRESHOLD = 0.03;
#pragma endregion

void ConfigurationManager::nextConfiguration() {
    controllerConfiguration++;
    if (controllerConfiguration > NUM_CONTROLLER_CONFIGS)
        controllerConfiguration = 1;
}

void ConfigurationManager::previousConfiguration() {
    controllerConfiguration--;
    if (controllerConfiguration < 1)
        controllerConfiguration = NUM_CONTROLLER_CONFIGS;
}

void ConfigurationManager::update(real_t commonThrust) {

    /* Only update the configuration if the common thrust is near zero. */
    if (commonThrust < THROTTLE_THRESHOLD)
        updateConfig();

    /* Update the wiggles in all cases. */
    updateWiggles();
}

void ConfigurationManager::updateConfig() {

    real_t tunerValue = getTuner();

    /* Only do configuration if we're in the changing zone. */
    if (tunerValue < CONFIG_UPPER_THRESHOLD &&
        tunerValue > CONFIG_LOWER_THRESHOLD)
        return;

    /* Only do configuration if the buzzer is not busy. */
    if (buzzerManager.isInstructionBusy())
        return;

    /* If we're waiting for the configuration to change. */
    if (this->isWaitingForConfigurationChange) {

        /* Change configuration if we've waited long enough. */
        if (getTime() - warningBeepsStartTime >= CONFIG_CHANGE_DELAY) {
            changedConfigurationTime              = getTime();
            this->isWaitingForConfigurationChange = false;
            if (tunerValue >= CONFIG_UPPER_THRESHOLD)
                nextConfiguration();
            else
                previousConfiguration();
            buzzerManager.addConfigurationBeeps(this->controllerConfiguration);
        }
    } else {

        /* Start playing warning beeps if we've waited long enough since the
           last configuration change. */
        if (getTime() - changedConfigurationTime >= AFTER_CONFIG_CHANGE_DELAY) {
            buzzerManager.addWarningBeeps();
            warningBeepsStartTime                 = getTime();
            this->isWaitingForConfigurationChange = true;
        }
    }
}

void ConfigurationManager::updateWiggles() {

    real_t tunerValue = getTuner();

    /* Only do wiggles if the buzzer is not busy. */
    if (buzzerManager.isInstructionBusy())
        return;

    /* Check if we've hit a threshold. */
    if (tunerValue >= WIGGLE_UPPER_THRESHOLD)
        isWiggleUpperThresholdHit = true;
    if (tunerValue <= WIGGLE_LOWER_THRESHOLD)
        isWiggleLowerThresholdHit = true;

    /* If we've hit both thresholds. */
    if (isWiggleUpperThresholdHit && isWiggleLowerThresholdHit) {
        isWiggleUpperThresholdHit = false;
        isWiggleLowerThresholdHit = false;
        currentNumWiggles++;

        /* If we've wiggled enough, sound the current configuration. */
        if (currentNumWiggles >= NUM_WIGGLES) {
            currentNumWiggles = 0;
            buzzerManager.addConfigurationBeeps(this->controllerConfiguration);
        }
    }

    /* Decrement wiggle counter from inactivity? */
    if (currentNumWiggles > 0) {
        if (getTime() - lastWiggleDecrementTime >= WIGGLE_DECREMENT_DELAY) {
            lastWiggleDecrementTime = getTime();
            currentNumWiggles--;
        }
    }
}
