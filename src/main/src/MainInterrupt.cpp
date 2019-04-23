#include "MainInterrupt.hpp"
#include "../sensors/ahrs/AHRS.hpp"
#include "../sensors/imu/IMU.hpp"
#include "../time/Time.hpp"
#include <MainFSM.hpp>  // Update control system


// Called by src-vivado every 238 Hz after initialization/calibration is complete.
void updateMainFSM() {

    // ! switch

    // ! CASE ( flightMode == MANUAL )
        // TODO: arming check
        // TODO: u_att = att.updateCtrl()
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

    /* Try updating the observers at 238 Hz. */
    
    // TODO: att.updateObserver()
    // TODO: alt.updateObserver()
    // TODO: pos.updateObserver()
    
}



void update() {

    // Test pin high to probe length of interrupt.
    writeValueToTestPin(true);

    // Keep the clock/timer up-to-date
    incrementTickCount();

    // IMU bias should be calculated before use.
    static bool isIMUCalibrated = false;
    // AHRS should calibrate with accelerometer before use.
    static bool isAHRSInitialized = false;

    // Phase 1: Calibrate IMU.
    if (!isIMUCalibrated) {
        isIMUCalibrated = calibrateIMU();
    }
    // Phase 2: Initialize AHRS. */
    else if (!isAHRSInitialized) {
        initAHRS();
        isAHRSInitialized = true;
    }
    // Phase 3: main operation
    else {
        updateMainFSM();
    }

    // Test pin low to probe length of interrupt.
    writeValueToTestPin(false);

}
