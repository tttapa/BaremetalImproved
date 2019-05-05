#pragma once
#include <Quaternion.hpp>
#include <real_t.h>

/**
 * Position (x,y) reference to track, consisting of two floats. This value
 * is measured in meters.
 */
struct PositionReference {
    real_t x; /* X position (m) */
    real_t y; /* Y position (m) */
};

/** 
 * Measurement from the Image Processing team, consisting of two floats
 * representing the global position in meters.
 */
struct PositionMeasurement {
    real_t x; /* X position (m) */
    real_t y; /* Y position (m) */
};

/**
 * Estimate of the state of the drone's position, consisting six components.
 * The first two floats are the quaternion components q1 and q2. The next two
 * floats represent the drone's global position, measured in meters. Finally,
 * two floats represent the horizontal velocity of the drone in m/s.
 */
struct PositionState {
    real_t q1; /* Orientation q1 component (/) */
    real_t q2; /* Orientation q2 component (/) */
    real_t x;  /* X position (m) */
    real_t y;  /* Y position (m) */
    real_t vx; /* X velocity (m/s) */
    real_t vy; /* Y velocity (m/s) */
};

/**
 * Integral of the error of the global position of the drone.
 */
struct PositionIntegralWindup {
    real_t x; /* X position (m) */
    real_t y; /* Y position (m) */
};

/**
 * Reference quaternion components q1 and q2 that will be sent to the
 * attitude controller.
 */
struct PositionControlSignal {
    real_t q1ref; /* Reference orientation q1 component (/) */
    real_t q2ref; /* Reference orientation q2 component (/) */
};

/**
 * Calculates the distance the two given positions in meters.
 * 
 * @param   position1
 *          first position
 * @param   position2
 *          second position
 * 
 * @return  the distance between the two given positions.
 */
real_t dist(PositionReference position1, PositionReference position2);

/**
 * Calculates the square of the distance the two given positions in meters.
 * 
 * @param   position1
 *          first position
 * @param   position2
 *          second position
 * 
 * @return  the square of distance between the two given positions.
 */
real_t distsq(PositionReference position1, PositionReference position2);

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

    /**
     * Calculate the current position control signal using the code generator.
     * 
     * @param   stateEstimate
     *          estimate of the current state, determined this cycle
     * @param   reference
     *          reference position to track
     * @param   integralWindup
     *          current integral windup
     * @param   droneConfiguration
     *          configuration of the drone
     * 
     * @return  the control signal to be sent to the attitude controller until
     *          the next measurement from the Image Processing team. The result
     *          only contains the quaternion components q1 and q2. The last
     *          component q3 should be determined by the anti-yaw-drift
     *          controller and from that the full quaternion should be
     *          constructed.
     */
    PositionControlSignal codegenControlSignal(
        PositionState stateEstimate, PositionReference reference,
        PositionIntegralWindup integralWindup, int droneConfiguration);

    /**
     * Calculate the current integral windup using the code generator.
     * 
     * @param   lastIntegralWindup
     *          integral windup from the last cycle
     * @param   reference
     *          reference position to track
     * 
     * @return  the current integral windup.
     */
    PositionIntegralWindup
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
     *          estimate of the previous state, determined last cycle
     * @param   measurement
     *          current measurement from the Image Processing team
     * @param   orientation
     *          current orientation of the drone
     * @param   droneConfiguration
     *          configuration of the drone
     * 
     * @return  the estimate of the current position state.
     */
    PositionState codegenCurrentStateEstimate(PositionState stateEstimate,
                                              PositionMeasurement measurement,
                                              Quaternion orientation,
                                              int droneConfiguration);

    /**
     * Clamp the given position control signal in [-0.0436,+0.0436].
     * 
     * @param   controlSignal
     *          control signal to clamp
     * 
     * @return  the clamped position control signal.
     */
    PositionControlSignal
    clampControlSignal(PositionControlSignal controlSignal);

  public:
    /**
     * Shift the position controller's estimate of the position by the given
     * correction.
     * 
     * @param   correctionX
     *          correction to be added to the x-coordinate of the estimate of
     *          the position controller
     * @param   correctionY
     *          correction to be added to the y-coordinate of the estimate of
     *          the position controller
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
     *          the reference position to track
     *
     * @return  the control signal to be sent to the attitude controller. The
     *          result only contains the quaternion components q1 and q2. The
     *          last component q3 should be determined by the anti-yaw-drift
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
     *          current orientation of the drone
     * @param   measurement
     *          new position measurement from the Image Processing team
     */
    void updateObserver(Quaternion orientation,
                        PositionMeasurement measurement);
};
