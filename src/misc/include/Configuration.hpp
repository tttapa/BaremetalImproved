// TODO: drone configurations vs controller configurations
/** Everything enabled on the drone. */
#define CONFIG_DEMO 0
/** Test 1. */
#define CONFIG_TEST1 1
/** Test 2. */
#define CONFIG_TEST2 2
/** Test 3. */
#define CONFIG_TEST3 3

/** Current drone configuration is CONFIG_DEMO. */
#define DRONE_CONFIG CONFIG_DEMO

/**
 * There are 5 controller configurations, the last of which being the
 * calibration mode.
 */
constexpr NUM_CONTROLLER_CONFIGS = 5;

/** The final controller configuration (5) is calibration mode. */
constexpr CALIBRATION_MODE = NUM_CONTROLLER_CONFIGS;

/**
 * Returns the current controller configuration.
 */
Configuration getControllerConfiguration();

/**
 * Resets the controller configuration to 1.
 */
void initControllerConfiguration();

/**
 * Increments the controller configuration, wrapping at 5.
 */
void nextControllerConfiguration();

/**
 * Decrements the controller configuration, wrapping at 1.
 */
void previousControllerConfiguration();
