#pragma once

/* Includes from src. */
#include <real_t.h>

/**
 * There are 5 controller configurations, the last of which being the
 * calibration mode.
 */
const int NUM_CONTROLLER_CONFIGS = 5;

/** The final controller configuration (5) is calibration mode. */
const int CALIBRATION_MODE = NUM_CONTROLLER_CONFIGS;

class ConfigurationManager {
  private:
    /** Time at which the configuration last changed. */
    float changedConfigurationTime;

    /** Current controller configuration in {1,2,...,NUM_CONTROLLER_CONFIGS}. */
    int controllerConfiguration;

    /** Number of wiggles accumulated by wiggling the tuner knob. */
    int currentNumWiggles;

    /**
     * Whether the tuner value is currently in the "changing configuration"
     * zone, waiting for the configuration to change.
     */
    bool isWaitingForConfigurationChange;

    /**
     * Whether the tuner value has passed the wiggle lower threshold.
     */
    bool isWiggleLowerThresholdHit;

    /**
     * Whether the tuner value has passed the wiggle upper threshold.
     */
    bool isWiggleUpperThresholdHit;

    /** Time at which the last wiggle was decremented. */
    float lastWiggleDecrementTime;

    /** Time at which the latest warning beeps started. */
    float warningBeepsStartTime;

    /**
     * Update the controller configuration based on the value of the RC tuner
     * knob.
     */
    void updateConfig();

    /**
     * Update the wiggle counter based on the value of the RC tuner knob and
     * sound the current controller configuration if there are enough wiggles.
     */
    void updateWiggles();

  public:
    /**
     * Returns the current controller configuration.
     */
    int getControllerConfiguration() { return this->controllerConfiguration; }

    /**
     * Resets the controller configuration to 1.
     */
    void init() { this->controllerConfiguration = 1; }

    /**
     * Increments the controller configuration, wrapping at 5.
     */
    void nextConfiguration();

    /**
     * Decrements the controller configuration, wrapping at 1.
     */
    void previousConfiguration();

    /**
     * Update the controller configuration based on the value of the RC tuner
     * knob. The configuration can be changed if the common thrust is near zero.
     * However, the wiggle will always work.
     * 
     * @param   commonThrust
     *          Current signal to be sent to the "common motor".
     */
    void update(real_t commonThrust);
};