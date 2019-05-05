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

/**
 * Shift the position controller's estimate of the position by the given
 * correction.
 * 
 * @param   correctionX
 *          correction to be added to the x-coordinate of the estimate of the
 *          position controller
 * @param   correctionY
 *          correction to be added to the y-coordinate of the estimate of the
 *          position controller
 */
void correctDronePosition(real_t correctionX, real_t correctionY);