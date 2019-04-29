#include <Altitude.hpp>
#include <Position.hpp>

/**
 * Output of the autonomous control system, which consists of a reference
 * position, reference height, whether the altitude controller should be
 * bypassed and which common thrust should be used if it is bypassed.
 */
struct AutonomousOutput {

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

    /** // TODO: trust accelerometer
     * If this is true, then the drone should trust the accelerometer's ax and
     * ay data to determine the position.
     */
    bool trustAccelerometerForPosition;

    /** Reference position to be sent to the position controller. */
    PositionReference referencePosition;
};

/** States present in the autonomous controller's finite state machine (FSM). */
enum AutonomousState {

    /** The drone is inactive on the ground. */
    IDLE_GROUND = 1,

    /** The drone is airborne, but the autonomous mode has not been started. */
    IDLE_AIR = 2,

    /** The drone is starting up the motors to prepare for takeoff. */
    PRE_TAKEOFF = 3,

    /** The drone is taking off in autonomous mode. */
    TAKEOFF = 4,

    /** The drone is attempting to hold its (x,y,z) position for 15 seconds. */
    LOITERING = 5,

    /** The drone is converging on its next destination. */
    CONVERGING = 6,

    /** The drone is navigating to its next destination. */
    NAVIGATING = 7,

    /** The drone has received the "landing flag" and is attempting to land. */
    LANDING = 8,

    /** The drone has activated the Wireless Power Transfer (WPT). */
    WPT = 9,

    /**
     * The drone has received a bad measurement or has not received any position
     * measurement in 2 seconds. It will set its reference orientation upright
     * and start beeping.
     */
    ERROR = 10,
};

/** States present in the QR finite state machine (FSM). */
enum QRState {

    /** No group is busy with QR codes. */
    QR_IDLE = 1,

    /** The Image Processing team is busy reading the QR code. */
    QR_READING = 2,

    /** The Cryptography team is busy decoding the QR code. */
    QR_CRYPTO_BUSY = 3,

    /** The Cryptography team has decoded a new target. */
    QR_NEW_TARGET = 4,

    /** The Cryptography team has decoded a landing instruction. */
    QR_LAND = 5,

    /** The Cryptography team has decoded an unknown instruction. */
    QR_UNKNOWN = 6,

    /** The Cryptography team could not decode the image sent to them. */
    QR_ERROR = 7

};

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
     * Seconds that have passed since the controller entered the current state.
     */
    real_t elapsedTime;

    /**
     * Current state of the autonomous controller's finite state machine (FSM).
     */
    AutonomousState autonomousState;

    /** Current state of the QR finite state machine (FSM). */
    QRState qrState;

    /** Most recent reference height of the autonomous controller. */
    AltitudeReference referenceHeight;

    /** Previous target position. */
    PositionReference previousTarget;

    /** Next target position. */
    PositionReference nextTarget;

    /** 

    /** Next QR code location. */
    PositionReference nextQRPosition;

    /**
     * Number of times the Cryptography team failed to decrypt the image sent by
     * the Image Processing team.
     */
    int qrErrorCount;

    /**
     * Counter for the number of tiles the drone has searched to find the QR
     * code.
     */
    int qrTilesSearched;

    /**
     * Calculates the next target in an outward spiral search for the QR code.
     * 
     * @return  the next target to check.
     */
    PositionReference getNextSearchTarget();

    /**
     * Returns whether the given position is a valid position to search.
     * 
     * @param   position
     *          position to check
     * 
     * @return  True if and only if the given position is within the bounds
     *          of the grid.
     */
    bool isValidSearchTarget(PositionReference position);

    /**
     * Set the current state of the autonomous controller's FSM to the given
     * state and reset the elapsed time.
     * 
     * @param   nextState
     *          new autonomous FSM state
     */
    void setAutonomousState(AutonomousState nextState);

    /**
     * Set this autonomous controller's "previousTarget" to the value of
     * "nextTarget", then set "nextTarget" equal to the given target.
     * 
     * @param   target
     *          new "nextTarget" for the autonomous controller
     */
    void setNextTarget(PositionReference target);

    /**
     * Set the current state of the QR FSM to the the given state (converted to
     * a QRState). If the conversion fails, the QR FSM state will become
     * QR_IDLE.
     * 
     * @param   nextState
     *          new QR FSM state
     */
    void setQRState(int nextState);


    /**
     * Update the autonomous controller's finite state machine (FSM). In the
     * resulting struct contains the next reference position and height,
     * possibly together with a bypass of the altitude controller.
     * 
     * @param   dronePosition
     *          current estimate of the drone's position
     * 
     * @return  the next AutonomousOutput.
     */
    AutonomousOutput updateAutonomousFSM(PositionReference dronePosition);


    /**
     * Update the autonomous controller's QR finite state machine (FSM) calculate
     * next reference position and height, possibly together with a bypass of
     * the altitude controller.
     */
    void updateQRFSM();

  public:
    /**
     * Reset the autonomous controller to the IDLE_AIR state and set the
     * reference position to the given position.
     * 
     * @param   currentPosition
     *          current position of the drone
     */
    void initAir(PositionReference currentPosition);

    /**
     * Reset the autonomous controller to the IDLE_GROUND state and set the
     * reference position to the given position.
     * 
     * @param   currentPosition
     *          current position of the drone
     */
    void initGround(PositionReference currentPosition);

    /**
     * Update the autonomous controller's QR finite state machine (FSM), then
     * update the autonomous FSM. The resulting struct contains the next
     * reference position and height, possibly together with a bypass of the
     * altitude controller.
     * 
     * @param   positionState
     *          current estimate of the position system state
     * 
     * @return  the next AutonomousOutput.
     */
    AutonomousOutput update(PositionState positionState);
};
