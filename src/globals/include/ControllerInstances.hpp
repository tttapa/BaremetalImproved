#pragma once
#include <Attitude.hpp>
#include <Altitude.hpp>
#include <Position.hpp>
#include <Autonomous.hpp>
#include <InputBias/InputBias.hpp>

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
 * Shift the position controller's estimate of the position by the given
 * correction.
 * 
 * @param   correctionX
 *          Correction to be added to the x-coordinate of the estimate of the
 *          position controller.
 * @param   correctionY
 *          Correction to be added to the y-coordinate of the estimate of the
 *          position controller.
 */
void correctDronePosition(real_t correctionX, real_t correctionY);

/**
 * Reset the controller instances.
 */
void initControllers();