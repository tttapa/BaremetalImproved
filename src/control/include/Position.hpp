#include <Altitude.hpp>
#include <Attitude.hpp>
#include <real_t.h>

/**
 * Position (x,y) reference to track, consisting of two floats. This value
 * is measured in meters.
 */
struct PositionReference {
    real_t x;  // X position (m)
    real_t y;  // Y position (m)
};

/** 
 * Measurement from the Image Processing team, consisting of two floats
 * representing the corrected global position in meters.
 */
struct PositionMeasurement {
    real_t x;  // X position (m)
    real_t y;  // Y position (m)
};

/**
 * Estimate of the state of the drone's position, consisting six components.
 * The first two floats are the quaternion components q1 and q2. The next two
 * floats represent the drone's corrected global position, measured in meters.
 * Finally two floats represent the horizontal velocity of the drone in m/s.
 */
struct PositionState {
    float q1;  // Orientation q1 component (/)
    float q2;  // Orientation q2 component (/)
    float x;   // X position (m)
    float y;   // Y position (m)
    float vx;  // X velocity (m/s)
    float vy;  // Y velocity (m/s)
};

/**
 * Integral of the error of the corrected global position of the drone.
 */
struct PositionIntegralWindup {
    real_t x;  // X position (m)
    real_t y;  // Y position (m)
};

/**
 * Reference quaternion components q1 and q2 that will be sent to the
 * attitude controller.
 */
struct PositionControlSignal {
    float q1ref;  // Reference orientation q1 component (/)
    float q2ref;  // Reference orientation q2 component (/)
};

/**
 * Class to control the position of the drone. The first part is an observer to
 * estimate the drone's quaternion components q1 and q2, global position and
 * horizontal velocity. Next, there is a controller to send the appropriate 
 * quaternion reference signal to the attitude controller based on how far the
 * drone's state estimate deviates from the reference state.
 *
 * To achieve this, the PositionController contains variables to store the
 * reference position, state estimate, integral windup and control signal.
 */
class PositionController {

  private:
    /**
     * Position reference to track, consisting of a two floats.
     */
    PositionReference reference;

    /**
     * Measurement from the Image Processing team, consisting of two floats
     * representing the corrected global position in meters.
     */
    PositionMeasurement measurement;

    /**
     * Estimate of the state of the drone's position, consisting six components.
     * The first two floats are the quaternion components q1 and q2. The next two
     * floats represent the drone's corrected global position, measured in meters.
     * Finally two floats represent the horizontal velocity of the drone in m/s.
     */
    PositionState stateEstimate;

    /**
     * Integral of the error of the corrected global position of the drone.
     */
    PositionIntegralWindup integralWindup;

    /**
     * Reference quaternion components q1 and q2 that will be sent to the
     * attitude controller.
     */
    PositionControlSignal controlSignal;

    /**
     * Boolean flag indicating whether there is a new measurement from the Image
     * Processing team during the current cycle (238 Hz).
     */
    //bool hasNewMeasurement;

    /**
     * Update the given position estimate using the code generator.
     * 
     * @param   stateEstimate
     *          estimate of the last position state, determined last cycle
     * @param   measurement
     *          current measurement from the Image Processing team
     * @param   orientation
     *          current orientation of the drone
     * @param   droneConfiguration
     *          configuration of the drone
     */
    void updateObserverCodegen(PositionState stateEstimate,
                               PositionMeasurement measurement,
                               AttitudeState orientation,
                               int droneConfiguration);

    /**
     * Update the given position control signal using the code generator.
     * 
     * @param   stateEstimate
     *          estimate of the current position state, determined last cycle
     * @param   reference
     *          height reference to track
     * @param   controlSignal
     *          control signal to update
     * @param   integralWindup
     *          integral windup to update
     * @param   droneConfiguration
     *          configuration of the drone
     */
    void updateControlSignalCodegen(PositionState stateEstimate,
                                    PositionReference reference,
                                    PositionControlSignal controlSignal,
                                    PositionIntegralWindup integralWindup,
                                    int droneConfiguration);

    // TODO: clamp where?
    void clampPositionControllerOutput(PositionControlSignal,
                                       PositionIntegralWindup);

    real_t qRefClamp;
    real_t blocksToMeters;

  public:
    /**
     * Check if the Image Processing team has sent a new measurement. The observer
     * and controller will only update this cycle (238 Hz) if there is a new
     * measurement. Therefore function should be called before trying to update the
     * observer or the controller.
     */
    //void checkForNewMeasurement();

    /**
     * Try updating the position observer (called at 238 Hz). This function will only
     * change the position estimate if there is a new measurement from the Image
     * Processing team. See PositionController::checkForNewMeasurement().
     */
    void updateObserver(AttitudeState, PositionMeasurement);

    /**
     * Try updating the position controller at 238 Hz. This function will only change
     * the position control signal if there is a new measurement from the Image
     * Processing team. See PositionController::checkForNewMeasurement().
     *
     * @return the control signal to be sent to the attitude controller. The result
     *         only contains the quaternion components q1 and q2. The last component
     *         q3 should be determined by the anti-yaw-drift controller and from that
     *         the full quaternion should be constructed.
     */
    PositionControlSignal updateControlSignal();

    void initializeController(AttitudeState, PositionMeasurement,
                              AltitudeMeasurement);
};
