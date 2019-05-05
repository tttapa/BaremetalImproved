#include "../include/ControllerInstances.hpp"

/** Instance of the attitude controller. */
AttitudeController attitudeController;

/** Instance of the altitude controller. */
AltitudeController altitudeController;

/** Instance of the position controller. */
PositionController positionController;

/** Instance of the autonomous controller. */
AutonomousController autonomousController;

void correctDronePosition(real_t correctionX, real_t correctionY) {
    positionController.correctPosition(correctionX, correctionY);
}