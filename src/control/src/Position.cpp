#include <Configuration.hpp>
#include <Globals.h>
#include <Position.hpp>
#include <SoftwareConstants.hpp>
#include <Time.hpp>

/* Use software constants from the POSITION namespace. */
using namespace POSITION;

PositionControlSignal
PositionController::clampControlSignal(PositionControlSignal controlSignal) {

    /* Load values from the position controller. */
    real_t q1ref = PositionController::controlSignal.q1ref;
    real_t q2ref = PositionController::controlSignal.q2ref;

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

void PositionController::updateObserver(Quaternion orientation,
                                        PositionMeasurement measurement) {

    /* Calculate time since last measurement in seconds. */
    real_t timeElapsed = getTime() - PositionController::lastMeasurementTime;

    /* Store the current time for the next cycle. */
    PositionController::lastMeasurementTime = getTime();

    /* Calculate the current state estimate. */
    PositionController::stateEstimate = codegenCurrentStateEstimate(
        PositionController::stateEstimate, measurement, orientation,
        getDroneConfiguration())
}

PositionControlSignal PositionController::updateControlSignal() {

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

void PositionController::init() {

    /* Reset the position controller. */
    PositionController::stateEstimate       = {};
    PositionController::integralWindup      = {};
    PositionController::controlSignal       = {};
    PositionController::lastMeasurementTime = 0.0;
}