#pragma once
#include <BaremetalCommunicationDef.hpp>
#include <Quaternion.hpp>
#include <real_t.h>

/** Highest valid x-coordinate. */
const real_t X_MAX = 8.0;

/** Lowest valid x-coordinate. */
const real_t X_MIN = -8.0;

/** Highest valid y-coordinate. */
const real_t Y_MAX = 4.0;

/** Lowest valid y-coordinate. */
const real_t Y_MIN = -4.0;

/**
z * Position (x,y) reference to track, consisting of two floats. This value
 * is measured in meters.
 */
struct PositionReference {
    real_t x;  ///< X position (m).
    real_t y;  ///< Y position (m).
};

/** 
 * Measurement from the Image Processing team, consisting of two floats
 * representing the global position in meters.
 */
struct PositionMeasurement {
    real_t x;  ///< X position (m).
    real_t y;  ///< Y position (m).
};

/**
 * Estimate of the state of the drone's position, consisting six components.
 * The first two floats are the quaternion components q1 and q2. The next two
 * floats represent the drone's global position, measured in meters. Finally,
 * two floats represent the horizontal velocity of the drone in m/s.
 */
struct PositionState {
    real_t q1;  ///< Orientation q1 component (/).
    real_t q2;  ///< Orientation q2 component (/).
    real_t x;   ///< X position (m).
    real_t y;   ///< Y position (m).
    real_t vx;  ///< X velocity (m/s).
    real_t vy;  ///< Y velocity (m/s).
};

/**
 * Integral of the error of the global position of the drone.
 */
struct PositionIntegralWindup {
    real_t x;  ///< X position (m).
    real_t y;  ///< Y position (m).
};

/**
 * Reference quaternion components q1 and q2 that will be sent to the
 * attitude controller.
 */
struct PositionControlSignal {
    real_t q1ref;  ///< Reference orientation q1 component (/).
    real_t q2ref;  ///< Reference orientation q2 component (/).
};

struct PositionStateBlind {
    real_t x;   ///< X position (m).
    real_t y;   ///< Y position (m).
    real_t vx;  ///< X velocity (m/s).
    real_t vy;  ///< Y velocity (m/s).
};

struct PositionControlSignalBlind {
    real_t q1;
    real_t q2;
};

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
real_t dist(Position position1, Position position2);

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
real_t distsq(Position position1, Position position2);

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
     * Estimate of the state of the drone's position, consisting six components.
     * The first two floats are the quaternion components q1 and q2. The next
     * two floats represent the drone's global position, measured in meters.
     * Finally, two floats represent the horizontal velocity of the drone
     * in m/s.
     */
    PositionState stateEstimate;

    /**
     * Integral of the error of the global position of the drone.
     */
    PositionIntegralWindup integralWindup;

    /**
     * Reference quaternion components q1 and q2 that will be sent to the
     * attitude controller.
     */
    PositionControlSignal controlSignal;

    /**
     * Time that the last measurement from the Image Processing team was
     * received (see Time.hpp).
     */
    real_t lastMeasurementTime;

  public:
    /**
     * Clamp the given position control signal in [-0.0436,+0.0436].
     * 
     * @param   controlSignal
     *          Control signal to clamp.
     * 
     * @return  The clamped position control signal.
     */
    static PositionControlSignal
    clampControlSignal(PositionControlSignal controlSignal);

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
     * Calculate the current integral windup using the code generator.
     * 
     * @param   lastIntegralWindup
     *          Integral windup from the last cycle.
     * @param   reference
     *          Reference position to track.
     * 
     * @return  The current integral windup.
     */
    static PositionIntegralWindup
    codegenIntegralWindup(PositionIntegralWindup integralWindup,
                          PositionReference reference,
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
        Quaternion orientation, real_t timeElapsed, int droneConfiguration);

    /**
     * Calculate the current position estimate using the code generator. This
     * function should be used during the first stage of takeoff and the second
     * stage of landing when the drone is too close to the ground to measure
     * the position.
     * 
     * @param   stateEstimateBlind
     *          Last four components of the position controller's state
     *          estimate, namely x, y, vx and vy.
     * @param   controlSignalBlind
     *          Struct containing the quaternion components q1 and q2 of the
     *          drone's orientation estimate.
     */
    static PositionState codegenCurrentStateEstimateBlind(
        PositionStateBlind stateEstimateBlind,
        PositionControlSignalBlind controlSignalBlind);

    /**
     * Shift the position controller's estimate of the position by the given
     * correction.
     * 
     * @param   correctionX
     *          Correction to be added to the x-coordinate of the estimate of
     *          the position controller.
     * @param   correctionY
     *          Correction to be added to the y-coordinate of the estimate of
     *          the position controller.
     */
    void correctPosition(real_t correctionX, real_t correctionY);

    /**
     * Reset the position controller.
     */
    void init();

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
     *          last component q3 should be determined by the anti-yaw-drift`
     *          controller and from that the full quaternion should be
     *          constructed.
     */
    PositionControlSignal updateControlSignal(PositionReference reference);

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
     * @param   currentTime
     *          Current time in seconds.
     * @param   measurement
     *          New position measurement from the Image Processing team.
     */
    void updateObserver(Quaternion orientation, real_t currentTime,
                        PositionMeasurement measurement);

    /**
     * Update the position observer 
     * 
     * @param   orientation
     *          Current orientation of the drone.
     */
    void updateObserverBlind(Quaternion orientation);
};
