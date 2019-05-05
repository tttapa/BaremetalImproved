#include <MainInterrupt.hpp>
#include <BaremetalCommunicationDef.hpp>
#include <Globals.hpp>
#include <SharedMemoryInstances.hpp>
#include <TiltCorrection.hpp>
#include "../../../src-vivado/sensors/ahrs/include/AHRS.hpp"
#include "../../../src-vivado/sensors/imu/include/IMU.hpp"
#include "../../../src-vivado/sensors/rc/include/RC.hpp"
#include "../../../src-vivado/sensors/sonar/include/Sonar.hpp"


// Called by src-vivado every 238 Hz after initialization/calibration is complete.
void updateMainFSM() {

    /* Previous flight mode initialized when the function is first called. */
    static FlightMode previousFlightMode = FlightMode::UNINITIALIZED;

    /* Read RC data and update the global struct. */
    setRCInput(readRC());

    /* Read IMU measurement and update the AHRS. */
    updateAHRS(readIMU());

    /* Read sonar measurement and correct it using the drone's orientation. */
    bool hasNewSonarMeasurement = readSonar();
    real_t sonarMeasurement;
    real_t correctedSonarMeasurement;
    if (hasNewSonarMeasurement) {
        sonarMeasurement = getFilteredSonarMeasurement();
        correctedSonarMeasurement = getCorrectedHeight(sonarMeasurement, attitudeController.getOrientationEstimate());
    }

    /* Read IMP measurement from shared memory and correct it using the sonar
       measurement and the drone's orientation. */
    bool hasNewIMPMeasurement = false;
    real_t yawMeasurement;
    Position correctedPositionMeasurement;
    if (visionComm->isDoneWriting()) {
        hasNewIMPMeasurement = true;
        VisionData visionData = visionComm->read();
        impPositionMeasurement = visionData.position;
        impYawMeasurement = visionData.yawAngle;
        correctedPositionMeasurement = getCorrectedPosition(ColVector<2>{VisionData.position.x, VisionData.position.y}, sonarMeasurement, attitudeController.getOrientationEstimate());
    }

    AttitudeControlSignal torqueMotorSignals;
    real_t commonThrust;
    switch(getRCFlightMode()) {
        case FlightMode::MANUAL:
            // TODO: arming check
            // TODO: gradual thrust change if last mode was altitude-hold
            commonThrust = getRCThrottle();
            // TODO: convert RC signal to quaternion
            torqueMotorSignals = attitudeController.updateControlSignal({}, commonThrust);
            break;
        case FlightMode::ALTITUDE_HOLD:
            if(hasNewSonarMeasurement) {
                
            }
            break;
        case FlightMode::AUTONOMOUS:
            break;
        case FlightMode::UNINITIALIZED:
            /* We will never get here because readRC() cannot return an
               uninitialized flight mode. */
            break;
    }



    // ! switch

    // ! CASE ( flightMode == MANUAL )
        // TODO: arming check
        // TODO: u_att = att.updateCtrl()
        // TODO: u_thr = getRCThrust()

    // ! CASE ( flightMode == ALTITDUE_HOLD )
        // TODO: if(*NEW_LOCATION_MEASUREMENT == 1) {
            // TODO: set PositionController::measurement
            // TODO: u_att = att.updateCtrl(getRCManualReference())
            // TODO: z_ref = altref.updateZRef(getRCThrust())   // Use throttle to move zref up or down
            // TODO: u_thr = alt.updateCtrl(z_ref)
        // TODO: }

    // ! CASE ( flightMode == AUTONOMOUS )
        // TODO: posref, altref = updateAutoFSM(getRCThrust(), getRCInductive())  // Call these as globals
        // TODO: qref = pos.updateCtrl(posref, measurement)
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
    
    // TODO: reset new measurement flags
    // TODO: *NEW_LOCATION_MEASUREMENT == 0

    /* Store flight mode. */
    previousFlightMode = getRCFlightMode();
    
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
