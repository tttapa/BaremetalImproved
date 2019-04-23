// Original: BareMetal/src/control/eagle1globals.h

enum DroneConfiguration {
    configurationDemo           = 1,
    configurationTestDemo       = 2,
    configurationTestLoiter     = 3,
    configurationTestTrajectory = 4,
    configurationCalibration    = 5
};

int currentDroneConfiguration;
int getDroneConfiguration();
void setDroneConfiguration(int configuration);