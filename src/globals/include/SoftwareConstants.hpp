#include <real_t.h>

namespace ATTITUDE {

/**
 * The largest control signal that can be sent to the "yaw torque motor" is
 * 0.10.
 */
const real_t getYawSignalClamp() { return 0.10; }

};  // namespace ATTITUDE

/**
 * Constants related to the altitude control system.
 */
namespace ALTITUDE {

/**
 * The largest marginal control signal that can be sent to the "common motor"
 * is 0.10.
 */
const real_t getMarginalSignalClamp() { return 0.10 }

/** The maximum height at which the drone may hover is 1.75 meters. */
const real_t getMaximumAltitudeReference() { return 1.75; }

/** The minimum height at which the drone may hover is 0.25 meters. */
const real_t getMinimumAltitudeReference() { return 0.25; }

/** The maximum speed of the reference height is 0.25 m/s. */
const real_t getRCReferenceMaxSpeed() { return 0.25; }

/** The threshold to start decreasing the reference height is 0.25. */
const real_t getRCReferenceLowerThreshold() { return 0.25; }

/** The threshold to start increasing the reference height is 0.75. */
const real_t getRCReferenceUpperThreshold() { return 0.75; }

}  // namespace ALTITUDE

/**
 * Constants related to the position control system.
 */
namespace POSITION {

/**
 * The largest reference quaternion component that can be sent to the attitude
 * control system is 0.0436.
 */
const real_t getReferenceQuaternionClamp() { return 0.0436; }

}  // namespace POSITION
