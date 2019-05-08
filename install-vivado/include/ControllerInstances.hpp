#pragma once
#include <Altitude.hpp>
#include <Attitude.hpp>
#include <Autonomous.hpp>
#include <InputBias/InputBias.hpp>
#include <Position.hpp>

/** Instance of the attitude controller. */
extern AttitudeController attitudeController;

/** Instance of the altitude controller. */
extern AltitudeController altitudeController;

/** Instance of the position controller. */
extern PositionController positionController;

/** Instance of the autonomous controller. */
extern AutonomousController autonomousController;

/** Instance of the input bias handler. */
extern InputBias inputBias;

/**
 * Initialize the controller instances.
 */
void initControllerInstances();