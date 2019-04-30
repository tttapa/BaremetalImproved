#pragma once
#include <Attitude.hpp>
#include <Altitude.hpp>
#include <Position.hpp>
#include <Autonomous.hpp>

/** Instance of the attitude controller. */
extern AttitudeController attitudeController;

/** Instance of the altitude controller. */
extern AltitudeController altitudeController;

/** Instance of the position controller. */
extern PositionController positionController;

/** Instance of the autonomous controller. */
extern AutonomousController autonomousController;