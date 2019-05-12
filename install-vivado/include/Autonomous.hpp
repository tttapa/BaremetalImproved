#pragma once

/* Includes from src. */
#include <Altitude.hpp>
#include <BaremetalCommunicationDef.hpp>  ///< QRFSMState, Position
#include <Position.hpp>

/**
 * Output of the autonomous control system, which consists of a reference
 * position, reference height, whether the altitude controller should be
 * bypassed and which common thrust should be used if it is bypassed.
 */
struct AutonomousOutput {
    AutonomousOutput(bool bypassAltitudeController,
                     AltitudeReference referenceHeight, real_t commonThrust,
                     bool updatePositionController,
                     bool trustAccelerometerForPosition,
                     Position referencePosition)
        : bypassAltitudeController{bypassAltitudeController},
          referenceHeight{referenceHeight}, commonThrust{commonThrust},
          updatePositionController{updatePositionController},
          trustAccelerometerForPosition{trustAccelerometerForPosition},
          referencePosition{referencePosition} {}

    /**
     * Whether the altitude controller should be bypassed. If this is true, then
     * this AutonomousOutput's commonThrust should be used instead of the
     * altitude controller's common thrust.
     */
    bool bypassAltitudeController;

    /** Reference height to be sent to the altitude controller. */
    AltitudeReference referenceHeight;

    /**
     * If bypassAltitudeController is true, then this value should be sent to
     * the "common motor".
     */
    real_t commonThrust;

    /** Whether the position controller should be updated. */
    bool updatePositionController;

    /**
     * If this is true, then the drone should trust the accelerometer's ax and
     * ay data to determine the position.
     */
    bool trustAccelerometerForPosition;

    /** Reference position to be sent to the position controller. */
    Position referencePosition;
};

/** States present in the autonomous controller's finite state machine (FSM). */
enum AutonomousState {

    /** The drone is inactive on the ground. */
    IDLE_GROUND = 0,

    /** The drone is airborne, but the autonomous mode has not been started. */
    IDLE_AIR = 1,

    /** The drone is starting up the motors to prepare for takeoff. */
    PRE_TAKEOFF = 2,

    /** The drone is taking off in autonomous mode. */
    TAKEOFF = 3,

    /** The drone is attempting to hold its (x,y,z) position for 15 seconds. */
    LOITERING = 4,

    /** The drone is converging on its next destination. */
    CONVERGING = 5,

    /** The drone is navigating to its next destination. */
    NAVIGATING = 6,

    /** The drone has received the "landing flag" and is attempting to land. */
    LANDING = 7,

    /** The drone has activated the Wireless Power Transfer (WPT). */
    WPT = 8,

    /**
     * The drone has received a bad measurement or has not received any position
     * measurement in 2 seconds. It will set its reference orientation upright
     * and start beeping.
     */
    ERROR = -1,
};

/**
 * Returns whether the given position is a valid position to search for the
 * QR code.
 * 
 * @param   position
 *          Position to check.
 * 
 * @return  True if and only if the given position is within the bounds
 *          of the grid.
 */
bool isValidSearchTarget(Position position);

/**
 * Class to control the autonomous navigation of the drone. It has one function
 * to update the finite state machine (FSM) which returns an AutonomousOutput.
 * The AutonomousOutput consists of a reference position, reference height,
 * whether the altitude controller should be bypassed and which common thrust
 * should be used if it is bypassed.
 * 
 * To achieve this, the AutonomousController contains variables to store the
 * the current autonomous FSM state, the current QR FSM state, the time that the
 * controller has been in that state, the most recent reference height, and the
 * previous/next target positions.
 */
class AutonomousController {

  private:
    /**
     * Current state of the autonomous controller's finite state machine (FSM).
     */
    AutonomousState autonomousState;

    /** Time that the autonomous controller entered its current state. */
    real_t autonomousStateStartTime = 0.0;

    /** Estimated time to navigate from previous target to next target. */
    real_t navigationTime = 0.0;

    /** Next QR code location. */
    Position nextQRPosition;

    /** Next target position. */
    Position nextTarget;

    /** Previous target position. */
    Position previousTarget;

    /**
     * Number of times the Cryptography team failed to decrypt the image sent by
     * the Image Processing team.
     */
    int qrErrorCount;

    /** Current state of the QR finite state machine (FSM). */
    QRFSMState qrState;

    /**
     * Counter for the number of tiles the drone has searched to find the QR
     * code.
     */
    int qrTilesSearched;

    /** Most recent reference height of the autonomous controller. */
    AltitudeReference referenceHeight;

    /**
     * Calculates the time since the autonomous controller entered its current
     * state in seconds.
     * 
     * @return  Seconds passed since the current autonomous state was entered.
     */
    real_t getElapsedTime();

    /**
     * Calculates the next target in an outward spiral search for the QR code.
     * 
     * @return  The next target to check.
     */
    Position getNextSearchTarget();

    /**
     * Set the current state of the autonomous controller's FSM to the given
     * state and reset the elapsed time.
     * 
     * @param   nextState
     *          New autonomous FSM state.
     */
    void setAutonomousState(AutonomousState nextState);

    /**
     * Set this autonomous controller's "previousTarget" to the value of
     * "nextTarget", then set "nextTarget" equal to the given target.
     * 
     * @param   target
     *          New "nextTarget" for the autonomous controller.
     */
    void setNextTarget(Position target);

    /**
     * Set the current state of the QR FSM to the the given state (converted to
     * a QRState). If the conversion fails, the QR FSM state will become
     * QR_IDLE.
     * 
     * @param   nextState
     *          New QR FSM state.
     */
    void setQRState(QRFSMState nextState);

    /**
     * Tell the autonomous controller's FSM to switch to the LANDING state. This
     * will be called from the QR FSM when the Cryptography team decodes a
     * QR_LAND instruction. The drone will either try to land at its current
     * position (e.g. if the pilot aborts the autonomous flight by setting the
     * throttle to zero) or at the most recent target position. This is based
     * on the parameter shouldLandAtCurrentPosition.
     * 
     * @param   shouldLandAtCurrentPosition
     *          Whether the landing target position should be the current drone
     *          position.
     * @param   currentPosition
     *          Current drone position.
     */
    void startLanding(bool shouldLandAtCurrentPosition,
                      Position currentPosition);

    /**
     * Tell the autonomous controller's FSM to switch to the NAVIGATING state
     * and start navigating to the given target. This will be called from the QR
     * FSM when the Cryptography team decodes a QR_NEW_TARGET instruction.
     * 
     * @param   nextQRPosition
     *          Position to navigate to, which will be the next QR code during
     *          autonomous mode.
     */
    void startNavigating(Position nextQRPosition);

    /**
     * Update the autonomous controller's finite state machine (FSM). In the
     * resulting struct contains the next reference position and height,
     * possibly together with a bypass of the altitude controller.
     * 
     * @param   currentPosition
     *          Current position of the drone.
     * 
     * @return  The next AutonomousOutput.
     */
    AutonomousOutput updateAutonomousFSM(Position currentPosition);

    /**
     * Update the autonomous controller's QR finite state machine (FSM) only
     * if the autonomous FSM state is CONVERGING. It is ANC's responsibility
     * to change the QR state if the FSM is currently in one of the following
     * states: QR_IDLE, QR_ERROR, QR_NEW_TARGET, QR_UNKNOWN, QR_LAND. If the 
     * FSM is in any other state, then this function will do nothing.
     */
    void updateQRFSM();

  public:
    /** Get the autonomous controller's autonomous state. */
    AutonomousState getAutonomousState() { return this->autonomousState; }

    /** Get the time that the autonomous controller entered its state. */
    real_t getAutonomousStateStartTime() { return autonomousStateStartTime; }
    /**
     * Get the autonomous controller's estimated time to navigate from its
     * previous target to its next target.
     */
    real_t getNavigationTime() { return navigationTime; }

    /** Get the next QR position. */
    Position getNextQRPosition() { return nextQRPosition; }

    /** Get the autonomous controller's next target position. */
    Position getNextTarget() { return nextTarget; }

    /** Get the autonomous controller's previous target position. */
    Position getPreviousTarget() { return previousTarget; }

    /**
     * Get the number of times the Cryptography team failed to decrypt the QR
     * code at the drone's current position.
     */
    int getQRErrorCount() { return qrErrorCount; }

    /** Get the current QR state. */
    QRFSMState getQRState() { return qrState; }

    /** Get the number of tiles the drone has searched to find the QR code. */
    int getQRTilesSearched() { return qrTilesSearched; }

    /** Get the autonomous controller's reference height. */
    real_t getReferenceHeight() { return referenceHeight.z; }

    /**
     * Reset the autonomous controller to the IDLE_AIR state and set the
     * reference position to the given position.
     * 
     * @param   currentPosition
     *          Current position of the drone.
     * @param   referenceHeight
     *          Reference height of the drone during autonomous mode.
     */
    void initAir(Position currentPosition, AltitudeReference referenceHeight);

    /**
     * Reset the autonomous controller to the IDLE_GROUND state and set the
     * reference position for takeoff and loitering to the given position.
     * 
     * @param   currentPosition
     *          Current position of the drone.
     */
    void initGround(Position currentPosition);

    /**
     * Update the autonomous controller's QR finite state machine (FSM), then
     * update the autonomous FSM. The resulting struct contains the next
     * reference position and height, possibly together with a bypass of the
     * altitude controller.
     * 
     * @param   currentPosition
     *          Current position of the drone.
     * 
     * @return  The next AutonomousOutput.
     */
    AutonomousOutput update(Position currentPosition);
};
