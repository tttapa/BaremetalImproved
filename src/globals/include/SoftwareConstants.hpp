#include <real_t.h>

namespace ATTITUDE {

/**
 * The largest control signal that can be sent to the "yaw torque motor" is
 * 0.10.
 */
real_t getYawSignalClamp() { return 0.10; }

}  // namespace ATTITUDE

/**
 * Constants related to the altitude control system.
 */
namespace ALTITUDE {

/**
 * The largest marginal control signal that can be sent to the "common motor"
 * is 0.10.
 */
real_t getMarginalSignalClamp() { return 0.10; }

/** The maximum height at which the drone may hover is 1.75 meters. */
real_t getMaximumReferenceHeight() { return 1.75; }

/** The minimum height at which the drone may hover is 0.25 meters. */
real_t getMinimumReferenceHeight() { return 0.25; }

/** The maximum speed of the reference height is 0.25 m/s. */
real_t getRCReferenceMaxSpeed() { return 0.25; }

/** The threshold to start decreasing the reference height is 0.25. */
real_t getRCReferenceLowerThreshold() { return 0.25; }

/** The threshold to start increasing the reference height is 0.75. */
real_t getRCReferenceUpperThreshold() { return 0.75; }

}  // namespace ALTITUDE

/**
 * Constants related to the autonomous control system.
 */
namespace AUTONOMOUS {

/**
 * If the drone is stays with 0.10 meters of its destination for a period of
 * time, then it will have converged on its target.
 */
real_t getConvergenceDistance() { return 0.10; }

/**
 * If the drone is stays with a certain distance of its destination for 1
 * seconds, then it will have converged on its target.
 */
real_t getConvergenceDuration() { return 1.0; }

/**
 * If the autonomous controller is in the state LOITERING, NAVIGATING or
 * CONVERGING, then the drone will land if the throttle value goes below
 * 0.05.
 */
real_t getLandingThrottle() { return 0.05; }

/**
 * The autonomous controller will loiter for 15 seconds before navigating.
 */
real_t getLoiterDuration() { return 15.0; }

/**
 * If the Cryptography team fails to decrypt the image sent by the Image
 * Processing team 3 times in a row, then it's likely that the drone is not
 * directly above the QR code. Therefore, it will start searching for it.
 */
int getMaxQRErrorCount() { return 3; }

// TODO: either 25 for radius 2 or 49 for radius 3
/**
 * If the Cryptography team fails to decrypt the image sent by the Image
 * Processing team too many times, then it's likely that the drone is not 
 * directly above the QR code. Therefore, it will start searching for it. After
 * 25 failed tiles (radius 2 around proposed QR position), the drone will stop
 * searching and attempt to land.
 */
int getMaxQRSearchCount() { return 25; }

/**
 * When the drone is navigating in autonomous mode, the reference will travel at
 * a speed of 0.5 m/s.
 */
real_t getNavigationSpeed() { return 0.5; }

/** The normal reference height for the drone in autonomous mode is 1 meter. */
real_t getReferenceHeight() { return 1.0; }

/**
 * The blind stage of the takeoff, meaning the sonar is not yet accurate,
 * lasts 0.5 seconds.
 */
real_t getTakeoffBlindDuration() { return 0.5; }

/**
 * During the blind stage of the takeoff, meaning the sonar is not yet accurate,
 * a marginal signal of 3% above the hovering signal will be sent to the "common
 * motor".
 */
real_t getTakeoffBlindMarginalThrust() { return 0.03; }

/** The entire takeoff will last 2 seconds. */
real_t getTakeoffDuration() { return 2.0; }

/**
 * If the autonomous controller is in the state IDLE_GROUND, then the drone
 * will take off if the throttle value exceeds 0.50.
 */
real_t getTakeoffThrottle() { return 0.50; }

}  // namespace AUTONOMOUS

/**
 * Constants related to the position control system.
 */
namespace POSITION {

/**
 * The largest reference quaternion component that can be sent to the attitude
 * control system is 0.0436.
 */
real_t getReferenceQuaternionClamp() { return 0.0436; }

/** Highest valid x-coordinate. */
real_t getXMax() { return 8.0; }

/** Lowest valid x-coordinate. */
real_t getXMin() { return -8.0; }

/** Highest valid y-coordinate. */
real_t getYMax() { return 4.0; }

/** Lowest valid y-coordinate. */
real_t getYMin() { return -4.0; }

}  // namespace POSITION
