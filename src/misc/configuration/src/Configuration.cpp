// Original: BareMetal/src/control/buzzercodesiel.c

#include <Configuration.hpp>

int getDroneConfiguration() {
    return currentDroneConfiguration;
}

void setDroneConfiguration(int configuration) {
    currentDroneConfiguration = configuration;
}