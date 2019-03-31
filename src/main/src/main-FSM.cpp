#include <Time.hpp>
#include <main-FSM.hpp>

static FlightMode flightMode = FlightMode::Manual;

void updateMainFSM() {
    IMUMeasurement imu = readIMU();
    Quat32 orientation = updateAHRS(imu);

    // read RC
    // read sonar
    // read position
    // eagle 1 actions?

    // remember last mode & check for mode switching
    switch (flightMode) {
        case (FlightMode::Manual):
            // arming check
            // u_att = att.updateCtrl(getRCManualReference())
            // u_thr = getRCThrust()
            break;
        case (FlightMode::AltitudeHold):
            // u_att = att.updateCtrl(getRCManualReference())
            // z_ref = altref.updateZRef(getRCThrust())   // Use throttle to move zref up or down
            // u_thr = alt.updateCtrl(z_ref)
            break;
        case (FlightMode::Autonomous):
            // posref, altref = updateAutoFSM(getRCThrust(), getRCInductive())  // Call these as globals
            // qref = pos.updateCtrl(posref)
            // u_att = att.updateCtrl(qref)
            // if altref.bypass, then u_thr = altref.thrust
            // else u_thr = alt.updateCtrl(altref.z_ref)
            break;
        case (FlightMode::Error):
            // TODO
            break;
    }

    // if isDroneKilled, then setMotors(0,0,0,0)
    // elseif gtc_busy, then setMotors(u_att, gtc_thrust)
    // else setMotors(u_att, u_thr)
    // att.updateObserver()
    // alt.updateObserver()
    // pos.updateObserver()

    // manual
    // altitude-hold
    // autonomous
    // error
}

FlightMode getFlightMode() { return flightMode; }