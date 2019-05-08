// Original: BareMetal/src/RC/RC.c
/**********************************************************************************************************************
*   Radio Control source file
*   This file contains all parameters used to read input from the RC.
*   This file should normally not be changed by the students.
*   Author: w. devries, p. coppens
***********************************************************************************************************************/
#include <RC.hpp>
#include "../../../main/src/PrivateHardwareConstants.hpp"
#include <xil_io.h>

/**
 * Scale throttle to [0.00, 0.80] so that we can still make attitude
 * adjustments when throttle reaches its highest value.
 */
const float MAX_THROTTLE = 0.80;

/* Address of the RC's throttle: PIN T14 (JD1). */
const int THROTTLE_ADDR = XPAR_RC_0_S00_AXI_BASEADDR;

/* Address of the RC's roll: PIN T15 (JD2). */
const int ROLL_ADDR = XPAR_RC_0_S00_AXI_BASEADDR + 0x04;

/* Address of the RC's pitch: PIN P14 (JD3). */
const int PITCH_ADDR = XPAR_RC_0_S00_AXI_BASEADDR + 0x08;

/* Address of the RC's yaw: PIN R14 (JD4). */
const int YAW_ADDR = XPAR_RC_0_S00_AXI_BASEADDR + 0x0C;

/* Address of the RC's flight mode: PIN U15 (JD6). */
const int FLIGHT_MODE_ADDR = XPAR_RC_1_S00_AXI_BASEADDR;

/* Address of the RC's WPT mode: PIN V17 (JD7). */
const int WPT_MODE_ADDR = XPAR_RC_1_S00_AXI_BASEADDR + 0x08;

/* Address of the RC's tuner knob: */
const int TUNER_ADDR = XPAR_RC_1_S00_AXI_BASEADDR + 0x0C;

/* Value if RC knob/joystick is at its lowest value. */
const float RC_LOW = 0.001109;

/* Value if RC knob/joystick is at its highest value. */
const float RC_HIGH = 0.001890;

/* Center value of the interval [RC_LOW, RC_HIGH]. */
const float RC_MID = (RC_LOW + RC_HIGH) / 2.0;

/* Size of RC knob/joystick deadzone. */
const float RC_MARGIN = (RC_HIGH - RC_LOW) / 40.0;

/* Value if RC knob/joystick is not available. */
const float RC_DEAD = 0.0;

// The last mode that the drone was in at the last interrupt.
int lastFlightMode = 0;
int lastWPTMode    = 0;

int newFlightModeCounter = 0;
int newWPTModeCounter    = 0;

/**
 * Reads the voltage from the given RC address. If RC_LOW and RC_HIGH are
 * defined correctly, then the result should be in [RC_LOW, RC_HIGH].
 */
float getRCValue(int address) {
    return (float) Xil_In32(address) / CLOCK_FREQUENCY;
}

/**
 * Returns the flight mode as a FlightMode.
 * 
 * @return  FlightMode::MANUAL
 *          If the value of the flight mode is in the first third of the RC
 *          range.
 * @return  FlightMode::ALTITUDE_HOLD
 *          If the value of the flight mode is in the second third of the RC
 *          range.
 * @return  FlightMode::AUTONOMOUS
 *          If the value of the flight mode is in the last third of the RC
 *          range.
 */
FlightMode getFlightMode(float flightModeValue) {

    float threshold1 = (RC_HIGH - RC_LOW) * 1.0 / 3.0;
    float threshold2 = (RC_HIGH - RC_LOW) * 2.0 / 3.0;

    // Get the current mode from RC
    int newFlightMode;
    if (flightModeValue < threshold1)
        newFlightMode = 0;
    else if (flightModeValue < threshold2)
        newFlightMode = 1;
    else
        newFlightMode = 2;

    // TODO: can this be smaller
    int MODE_DELAY = 50;
    // Add the delay for a mode switch
    if (newFlightMode != lastFlightMode) {
        newFlightModeCounter++;
        if (newFlightModeCounter > MODE_DELAY) {
            newFlightModeCounter = 0;
            lastFlightMode       = newFlightMode;
            return newFlightMode;
        }
    } else {
        newFlightModeCounter = 0;
    }
    return lastFlightMode;
}

/**
 * Returns the Wireless Power Transfer mode as a WPTMode.
 * 
 * @return  WPTMode::OFF
 *          If the value of the flight mode is in the first half of the RC
 *          range.
 * @return  WPTMode::ON
 *          If the value of the flight mode is in the second half of the RC
 *          range.
 */
WPTMode getWPTMode(float wptValue) {

    int newWPTMode;
    if (wptValue < RC_MID)
        newWPTMode = 0;
    else
        newWPTMode = 1;

    // TODO: can this be smaller
    int MODE_DELAY = 50;
    // Add the delay for a mode switch
    if (newWPTMode != lastWPTMode) {
        newWPTModeCounter++;
        if (newWPTModeCounter > MODE_DELAY) {
            newWPTModeCounter = 0;
            lastWPTMode       = newWPTMode;
            return newWPTMode;
        }
    } else {
        newWPTModeCounter = 0;
    }
    return lastWPTMode;
}

/**
 * Clamp value in [RC_LOW, RC_HIGH].
 * 
 *  @return clamped value.
 */
float clamp(float x) {
    if (x < RC_LOW)
        return RC_LOW;
    if (x > RC_HIGH)
        return RC_HIGH;
    return x;
}

/**
 * Clamp value in [RC_LOW, RC_HIGH]. If value is not in this interval, it will
 * be set to RC_MID.
 * 
 *  @return clamped value.
 */
float clampMid(float x) {
    // TODO:
    //       x in (-inf, LOW-MARGIN) -> MID
    //       x in [LOW-MARGIN, LOW) -> LOW
    //       x in [LOW, HIGH] -> x
    //       x in (HIGH, HIGH+MARGIN] -> HIGH
    //       x in (HIGH+MARGIN, inf) -> MID
    if (x < RC_LOW - RC_MARGIN || x > RC_HIGH + RC_MARGIN)
        return RC_MID;
    return x;
}

/**
 * Rescale to [0, 1] using deadzone at RC_LOW.
 * 
 *  @return rescaled value.
 */
float rescale(float x) {

    /* Upper edge of deadzone. */
    // TODO: change this to "RC_DEADZONE_SIZE"
    float low = RC_LOW + RC_MARGIN;

    /* Out of range. */
    if (x < low)
        return 0.0;
    if (x > RC_HIGH)
        return 1.0;

    /* In range. */
    return (x - low) / (RC_HIGH - low);
}

/**
 * Rescale to [-0.5, 0.5] using deadzone at RC_MID.
 * 
 *  @return rescaled value.
 */
float rescaleMid(float x) {

    /* Edges of deadzone. */
    // TODO: change this to "RC_DEADZONE_SIZE"
    float midlow  = RC_MID - RC_MARGIN;
    float midhigh = RC_MID + RC_MARGIN;

    /* Out of range. */
    if (x < RC_LOW)
        return -0.5;
    if (x > RC_HIGH)
        return 0.5;

    /* In range. */
    if (x < midlow)
        return (x - midlow) / (midlow - RC_LOW);
    if (x > midhigh)
        return (x - midhigh) / (RC_HIGH - midhigh);

    /* Deadzone. */
    return 0;
}

RCValues readRC() {

    /* Get all of the RC controls. */
    float throttle = rescale(clamp(getRCValue(THROTTLE_ADDR)));
    float pitch    = rescaleMid(clampMid(getRCValue(PITCH_ADDR)));
    float roll     = rescaleMid(clampMid(getRCValue(ROLL_ADDR)));
    float yaw      = rescaleMid(clampMid(getRCValue(YAW_ADDR)));
    // TODO: dead should stay mid, but margin should increase
    float tuner           = rescaleMid(clampMid(getRCValue(TUNER_ADDR)));
    float flightModeValue = rescale(clamp(getRCValue(FLIGHT_MODE_ADDR)));
    float wptValue        = rescale(clamp(getRCValue(WPT_MODE_ADDR)));

    throttle = throttle * MAX_THROTTLE;

    return RCInput{
        throttle,
        pitch,
        roll,
        yaw,
        tuner,
        getFlightMode(flightModeValue),
        getWPTMode(wptValue),
    };
}
