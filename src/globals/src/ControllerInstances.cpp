#include "../include/ControllerInstances.hpp"

/** Instance of the attitude controller. */
AttitudeController attitudeController;

/** Instance of the altitude controller. */
AltitudeController altitudeController;

/** Instance of the position controller. */
PositionController positionController;

/** Instance of the autonomous controller. */
AutonomousController autonomousController;

/** Instance of the input bias handler. */
InputBias inputBias;

void initControllers() {
    attitudeController.init();
    altitudeController.init();
    positionController.init();
    autonomousController.initGround({});
    inputBias.init();
}