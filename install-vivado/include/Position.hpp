#pragma once

/* Includes from src. */
#include <BaremetalCommunicationDef.hpp>  ///< VisionPosition
#include <LoggerStructs.hpp>
#include <Quaternion.hpp>
#include <Vector.hpp>

/* Use "Position" for readability: actual type is Vec2f. */
using Position = Vec2f;

/* Use "HorizontalVelocity" for readability: actual type is Vec2f. */
using HorizontalVelocity = Vec2f;

/** Blocks to meters. */
const float BLOCKS_TO_METERS = 0.30;

/** Meters to blocks. */
const float METERS_TO_BLOCKS = 1.0 / BLOCKS_TO_METERS;

/** Highest valid x-coordinate in blocks. */
const float X_MAX_BLOCKS = 9.5;

/** Lowest valid x-coordinate in blocks. */
const float X_MIN_BLOCKS = -0.5;

/** Center x-coordinate in blocks. */
const float X_CENTER_BLOCKS = (X_MIN_BLOCKS + X_MAX_BLOCKS) / 2.0;

/** Highest valid y-coordinate in blocks. */
const float Y_MAX_BLOCKS = 9.5;

/** Lowest valid y-coordinate in blocks. */
const float Y_MIN_BLOCKS = -0.5;

/** Center y-coordinate in blocks. */
const float Y_CENTER_BLOCKS = (Y_MIN_BLOCKS + Y_MAX_BLOCKS) / 2.0;

/** Highest valid x-coordinate. */
const float X_MAX = X_MAX_BLOCKS * BLOCKS_TO_METERS;

/** Lowest valid x-coordinate. */
const float X_MIN = X_MIN_BLOCKS * BLOCKS_TO_METERS;

/** Highest valid y-coordinate. */
const float Y_MAX = Y_MAX_BLOCKS * BLOCKS_TO_METERS;

/** Lowest valid y-coordinate. */
const float Y_MIN = Y_MIN_BLOCKS * BLOCKS_TO_METERS;

/**
 * Calculates the distance the two given positions in meters.
 * 
 * @param   position1
 *          First position.
 * @param   position2
 *          Second position.
 * 
 * @return  the distance between the two given positions.
 */
float dist(Position position1, Position position2);

/**
 * Calculates the square of the distance the two given positions in meters.
 * 
 * @param   position1
 *          First position.
 * @param   position2
 *          Second position.
 * 
 * @return  the square of distance between the two given positions.
 */
float distsq(Position position1, Position position2);

/**
 * Class to control the position of the drone. The first part is an observer to
 * estimate the drone's quaternion components q1 and q2, global position and
 * horizontal velocity. Next, there is a controller to send the appropriate 
 * quaternion reference signal to the attitude controller based on how far the
 * drone's state estimate deviates from the reference state.
 *
 * To achieve this, the PositionController contains variables to store the
 * state estimate, integral windup, control signal and the last measurement
 * time.
 */
class PositionController {

  private:
    /**
     * Reference quaternion components q1 and q2 that will be sent to the
     * attitude controller.
     */
    PositionControlSignal controlSignal;

    /**
     * Integral of the error of the global position of the drone.
     */
    PositionIntegralWindup integralWindup;

    /**
     * Time that the last measurement from the Image Processing team was
     * received (see Time.hpp).
     */
    float lastMeasurementTime = 0.0;

    /** Position measurement from IMP (or accelerometer data). */
    PositionMeasurement measurement;

    /** Reference position (x,y), which is stored to pass on to the logger. */
    PositionReference reference;

    /**
     * Estimate of the state of the drone's position, consisting six components.
     * The first two floats are the quaternion components q1 and q2. The next
     * two floats represent the drone's global position, measured in meters.
     * Finally, two floats represent the horizontal velocity of the drone
     * in m/s.
     */
    PositionState stateEstimate;

  public:
    /**
     * Clamp the current position control signal in [-0.0436,+0.0436].
     */
    void clampControlSignal();

    /**
     * Calculate the current position control signal using the code generator.
     * 
     * @param   stateEstimate
     *          Estimate of the current state, determined this cycle.
     * @param   reference
     *          Reference position to track.
     * @param   integralWindup
     *          Current integral windup.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The control signal to be sent to the attitude controller until
     *          the next measurement from the Image Processing team. The result
     *          only contains the quaternion components q1 and q2. The last
     *          component q3 should be determined by the anti-yaw-drift
     *          controller and from that the full quaternion should be
     *          constructed.
     */
    static PositionControlSignal codegenControlSignal(
        PositionState stateEstimate, PositionReference reference,
        PositionIntegralWindup integralWindup, int droneConfiguration);

    /**
     * Calculate the current blind position control signal using the code
     * generator.
     * 
     * @param   stateEstimate
     *          Estimate of the current state, determined this cycle.
     * @param   reference
     *          Reference position to track.
     * @param   integralWindup
     *          Current integral windup.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The control signal to be sent to the attitude controller until
     *          the next measurement from the Image Processing team. The result
     *          only contains the quaternion components q1 and q2. The last
     *          component q3 should be determined by the anti-yaw-drift
     *          controller and from that the full quaternion should be
     *          constructed.
     */
    static PositionControlSignal codegenControlSignalBlind(
        PositionState stateEstimate, PositionReference reference,
        PositionIntegralWindup integralWindup, int droneConfiguration);

    /**
     * Calculate the current integral windup using the code generator.
     * 
     * @param   lastIntegralWindup
     *          Integral windup from the last cycle.
     * @param   reference
     *          Reference position to track.
     * @param   stateEstimate
     *          Estimate of the current state, determined this cycle.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The current integral windup.
     */
    static PositionIntegralWindup
    codegenIntegralWindup(PositionIntegralWindup integralWindup,
                          PositionReference reference,
                          PositionState stateEstimate, int droneConfiguration);

    /**
     * Calculate the current (blind) integral windup using the code generator.
     * 
     * @param   lastIntegralWindup
     *          Integral windup from the last cycle.
     * @param   reference
     *          Reference position to track.
     * @param   stateEstimate
     *          Estimate of the current state, determined this cycle.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The current integral windup.
     */
    static PositionIntegralWindup codegenIntegralWindupBlind(
        PositionIntegralWindup integralWindup, PositionReference reference,
        PositionState stateEstimate, int droneConfiguration);

    /**
     * Calculate the current position estimate using the code generator. Because
     * the position control system is implemented with a varying sample time,
     * this function should be called before PositionController::
     * codegenControlSignal() is called in order to determine the state estimate
     * for the current cycle.
     * 
     * @param   stateEstimate
     *          Estimate of the previous state, determined last cycle.
     * @param   measurement
     *          Current measurement from the Image Processing team.
     * @param   orientation
     *          Current orientation of the drone.
     * @param   timeElapsed
     *          Time elapsed in seconds since the last update of the position
     *          controller's state estimate.
     * @param   droneConfiguration
     *          Configuration of the drone.
     * 
     * @return  The estimate of the current position state.
     */
    static PositionState codegenCurrentStateEstimate(
        PositionState stateEstimate, PositionMeasurement measurement,
        Quaternion orientation, float timeElapsed, int droneConfiguration);

    /**
     * Calculate the current position estimate using the code generator. This
     * function should be used during the first stage of takeoff and the second
     * stage of landing when the drone is too close to the ground to measure
     * the position.
     * 
     * @param   stateEstimate
     *          Last estimate of the position state.
     * @param   controlSignal
     *          Struct containing the quaternion components q1 and q2 of the
     *          drone's orientation estimate.
     * @param   orientation
     *          Current orientation of the drone.
     */
    static PositionState
    codegenCurrentStateEstimateBlind(PositionState stateEstimate,
                                     PositionControlSignal controlSignal,
                                     Quaternion orientation);

    /** Get the position controller's control signal. */
    PositionControlSignal getControlSignal() { return this->controlSignal; }

    /** Get the position controller's integral windup. */
    PositionIntegralWindup getIntegralWindup() { return this->integralWindup; }

    /** Get the position controller's last measurement time. */
    float getLastMeasurementTime() { return this->lastMeasurementTime; }

    /** Get the position controller's measurement. */
    PositionMeasurement getMeasurement() { return this->measurement; }

    /** Get the position controller's reference. */
    PositionReference getReference() { return this->reference; }

    /** Get the position controller's reference position. */
    Position getReferencePosition() { return this->reference.p; }

    /** Get the position controller's state estimate. */
    PositionState getStateEstimate() { return this->stateEstimate; }

    /**
     * Reset the position controller.
     * 
     * @param   currentPosition
     *          Current position of the drone.
     */
    void init(Position currentPosition);

    /**
     * Correct the position controller's estimate with the given position. The
     * position component of the estimate will jump to the correct square.
     * 
     * @param   correctPosition
     *          Correct position, read from the QR code, in blocks.
     */
    void correctPositionEstimateBlocks(Position correctPosition);

    /**
     * Correct the position controller's estimate with the given position. The
     * position component of the estimate will jump to the correct square.
     * 
     * @param   correctPosition
     *          Correct position, read from the QR code, in blocks.
     */
    void correctPositionEstimateBlocks(VisionPosition correctPosition);

    /**
     * Update the position controller with the given reference position. This
     * function should only be called when there is a new measurement from the
     * Image Processing team.
     * 
     * @param   reference
     *          The reference position to track.
     *
     * @return  The control signal to be sent to the attitude controller. The
     *          result only contains the quaternion components q1 and q2. The
     *          last component q3 should be determined by the anti-yaw-drift
     *          controller and from that the full quaternion should be
     *          constructed.
     */
    PositionControlSignal updateControlSignal(PositionReference reference);

    /**
     * Update the blind position controller with the given reference position.
     * This function should be called at the IMU's frequency during the blind
     * stages of takeoff and landing.
     * 
     * @param   reference
     *          The reference position to track.
     *
     * @return  The control signal to be sent to the attitude controller. The
     *          result only contains the quaternion components q1 and q2. The
     *          last component q3 should be determined by the anti-yaw-drift
     *          controller and from that the full quaternion should be
     *          constructed.
     */
    PositionControlSignal updateControlSignalBlind(PositionReference reference);

    /**
     * Update the position observer with the given measurement position and the
     * current orientation of the drone. This function should only be called
     * when there is a new measurement from the Image Processing team. Because
     * the position control system is implemented with a varying sample time,
     * this function should be called before PositionController::
     * updateControlSignal() is called in order to determine the state estimate
     * for the current cycle.
     * 
     * @param   orientation
     *          Current orientation of the drone.
     * @param   measurement
     *          New position measurement from the Image Processing team.
     */
    void updateObserver(Quaternion orientation,
                        PositionMeasurement measurement);

    /**
     * Update the position observer 
     * 
     * @param   orientation
     *          Current orientation of the drone.
     */
    void updateObserverBlind(Quaternion orientation);
};
