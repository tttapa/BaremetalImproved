#include <ControllerInstances.hpp>

/** Instance of the attitude controller. */
AttitudeController attitudeController;

/** Instance of the altitude controller. */
AltitudeController altitudeController;

/** Instance of the position controller. */
PositionController positionController;

/** Instance of the autonomous controller. */
AutonomousController autonomousController;

void initControllerInstances() {
    attitudeController.init();
    altitudeController.init(0.0);
    positionController.init({});
    autonomousController.initGround({});
}