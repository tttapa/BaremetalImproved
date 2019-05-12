#pragma once

/* Includes from src. */
#include <Altitude.hpp>
#include <Attitude.hpp>
#include <Autonomous.hpp>
#include <Position.hpp>

/** Instance of the attitude controller. */
extern AttitudeController attitudeController;

/** Instance of the altitude controller. */
extern AltitudeController altitudeController;

/** Instance of the position controller. */
extern PositionController positionController;

/** Instance of the autonomous controller. */
extern AutonomousController autonomousController;

/**
 * Initialize the controller instances.
 */
void initControllerInstances();