
// TODO: remember last mode & check for mode switching

void updateMainFSM() {

    // ! switch

    // ! CASE ( flightMode == MANUAL )
        // TODO: arming check
        // TODO: u_att = att.updateCtrl(getRCManualReference())
        // TODO: u_thr = getRCThrust()

    // ! CASE ( flightMode == ALTITDUE_HOLD )
        // TODO: u_att = att.updateCtrl(getRCManualReference())
        // TODO: z_ref = altref.updateZRef(getRCThrust())   // Use throttle to move zref up or down
        // TODO: u_thr = alt.updateCtrl(z_ref)

    // ! CASE ( flightMode == AUTONOMOUS )
        // TODO: posref, altref = updateAutoFSM(getRCThrust(), getRCInductive())  // Call these as globals
        // TODO: qref = pos.updateCtrl(posref)
        // TODO: u_att = att.updateCtrl(qref)
        // TODO: if altref.bypass, then u_thr = altref.thrust
        // TODO: else u_thr = alt.updateCtrl(altref.z_ref)

    // ! CASE ( flightMode == ERROR )
        // TODO: why is this here again?
        
    // ! end switch


    // TODO: if isDroneKilled, then setMotors(0,0,0,0)
    // TODO: elseif gtc_busy, then setMotors(u_att, gtc_thrust)
    // TODO: else setMotors(u_att, u_thr)
    // TODO: att.updateObserver()
    // TODO: alt.updateObserver()
    // TODO: pos.updateObserver()
    
}