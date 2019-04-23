// Original: BareMetal/src/control/buzzercodesiel.c

#include <configuration.hpp>

int getDroneConfiguration() {
    return currentDroneConfiguration;
}

void setDroneConfiguration(int configuration) {
    currentDroneConfiguration = configuration;
}