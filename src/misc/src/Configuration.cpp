#include <Configuration.hpp>

/** Current controller configuration in {1,2,...,NUM_CONTROLLER_CONFIGS}. */
static int controllerConfiguration = 1;

int getControllerConfiguration() { return controllerConfiguration; }

void initControllerConfiguration() { controllerConfiguration = 1; }

void nextControllerConfiguration() {
    controllerConfiguration++;
    if (controllerConfiguration > NUM_CONTROLLER_CONFIGS)
        controllerConfiguration = 1;
}

void previousControllerConfiguration() {
    controllerConfiguration--;
    if (controllerConfiguration < 1)
        controllerConfiguration = NUM_CONTROLLER_CONFIGS;
}
