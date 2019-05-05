#include <Configuration.hpp>
#include <Globals.hpp>
#include <Position.hpp>
#include <SoftwareConstants.hpp>
#include <Time.hpp>

/* Use software constants from the POSITION namespace. */
using namespace POSITION;

real_t dist(PositionReference position1, PositionReference position2) {
    return std::sqrt(distsq(position1, position2));
}

real_t distsq(PositionReference position1, PositionReference position2) {
    real_t dx = position2.x - position1.x;
    real_t dy = position2.y - position1.y;
    return dx * dx + dy * dy;
}

PositionControlSignal
PositionController::clampControlSignal(PositionControlSignal controlSignal) {

    /* Load values from the position controller. */
    real_t q1ref = controlSignal.q1ref;
    real_t q2ref = controlSignal.q2ref;

    /* Clamp q1ref and q2ref. */
    if (q1ref > getReferenceQuaternionClamp())
        q1ref = getReferenceQuaternionClamp();
    if (q1ref < -getReferenceQuaternionClamp())
        q1ref = -getReferenceQuaternionClamp();
    if (q2ref > getReferenceQuaternionClamp())
        q2ref = getReferenceQuaternionClamp();
    if (q2ref < -getReferenceQuaternionClamp())
        q2ref = -getReferenceQuaternionClamp();

    return PositionControlSignal{q1ref, q2ref};
}

void PositionController::correctPosition(real_t correctionX,
                                         real_t correctionY) {
    PositionController::stateEstimate.x += correctionX;
    PositionController::stateEstimate.y += correctionY;
}

void PositionController::init() {
    /* Reset the position controller. */
    PositionController::stateEstimate       = {};
    PositionController::integralWindup      = {};
    PositionController::controlSignal       = {};
    PositionController::lastMeasurementTime = 0.0;
}

PositionControlSignal
PositionController::updateControlSignal(PositionReference reference) {

    /* Calculate integral windup. */
    PositionController::integralWindup =
        codegenIntegralWindup(PositionController::integralWindup, reference);

    /* Calculate control signal (unclamped). */
    PositionController::controlSignal = codegenControlSignal(
        PositionController::stateEstimate, reference,
        PositionController::integralWindup, getDroneConfiguration());

    /* Clamp control signal. */
    PositionController::controlSignal =
        clampControlSignal(PositionController::controlSignal);

    return PositionController::controlSignal;
}

void PositionController::updateObserver(Quaternion orientation,
                                        PositionMeasurement measurement) {
    /* Calculate the current state estimate. */
    PositionController::stateEstimate = codegenCurrentStateEstimate(
        PositionController::stateEstimate, measurement, orientation,
        getDroneConfiguration());
}