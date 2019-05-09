#include <sensors/RC.hpp>

/* Includes from src-vivado. */
#include "../PrivateHardwareConstants.hpp"

/* Includes from Xilinx. */
#include <xil_io.h>

/** Address of the RC's throttle: PIN T14 (JD1). */
uint32_t *const THROTTLE_ADDR = (uint32_t *) XPAR_RC_0_S00_AXI_BASEADDR;

/** Address of the RC's roll: PIN T15 (JD2). */
uint32_t *const ROLL_ADDR = ((uint32_t *) XPAR_RC_0_S00_AXI_BASEADDR) + 1;

/** Address of the RC's pitch: PIN P14 (JD3). */
uint32_t *const PITCH_ADDR = ((uint32_t *) XPAR_RC_0_S00_AXI_BASEADDR) + 2;

/** Address of the RC's yaw: PIN R14 (JD4). */
uint32_t *const YAW_ADDR = ((uint32_t *) XPAR_RC_0_S00_AXI_BASEADDR) + 3;

/** Address of the RC's flight mode: PIN U15 (JD6). */
uint32_t *const FLIGHT_MODE_ADDR = (uint32_t *) XPAR_RC_1_S00_AXI_BASEADDR;

/** Address of the RC's WPT mode: PIN V17 (JD7). */
uint32_t *const WPT_MODE_ADDR = ((uint32_t *) XPAR_RC_1_S00_AXI_BASEADDR) + 2;

/** Address of the RC's tuner knob: */
uint32_t *const TUNER_ADDR = ((uint32_t *) XPAR_RC_1_S00_AXI_BASEADDR) + 3;

/** Value if RC knob/joystick is at its lowest value. */
const float RC_LOW = 0.001109;

/** Value if RC knob/joystick is at its highest value. */
const float RC_HIGH = 0.001890;

/** Center value of the interval [RC_LOW, RC_HIGH]. */
const float RC_MID = (RC_LOW + RC_HIGH) / 2.0;

/**
 * During the "mid scaling", values that are outside of [RC_LOW, RC_HIGH] by
 * less than 20% of that interval's size are still valid. They will be mapped to
 * either RC_LOW or RC_HIGH.
 */
const float RC_MARGIN = (RC_HIGH - RC_LOW) / 5.0;

/** Size of RC knob/joystick deadzone. */
const float RC_DEADZONE = (RC_HIGH - RC_LOW) / 40.0;

/** Value if RC knob/joystick is not available. */
const float RC_DEAD = 0.0;

/* The last mode that the drone was in at the last interrupt. */
FlightMode lastFlightMode = FlightMode::UNINITIALIZED;
WPTMode lastWPTMode       = WPTMode::OFF;
int newFlightModeCounter  = 0;
int newWPTModeCounter     = 0;

/**
 * Reads the voltage from the given RC address. If RC_LOW and RC_HIGH are
 * defined correctly, then the result should be in [RC_LOW, RC_HIGH].
 * 
 * @param   address
 *          RC address to read from.
 * 
 * @return  Value of the given address in [RC_LOW, RC_HIGH] (approximately).
 */
float getRCValue(uint32_t *address) {
    return (float) Xil_In32((uintptr_t) address) / CLOCK_FREQUENCY;
}

/**
 * Returns the flight mode as a FlightMode.
 * 
 * @param   flightModeValue
 *          The RC value from which the flight mode will be determined.
 * 
 * @return  FlightMode::MANUAL
 *          If the given value is in the first third of the interval
 *          [RC_LOW, RC_HIGH].
 * @return  FlightMode::ALTITUDE_HOLD
 *          If the given value is in the second third of the interval
 *          [RC_LOW, RC_HIGH].
 * @return  FlightMode::AUTONOMOUS
 *          If the given value is in the last third of the interval
 *          [RC_LOW, RC_HIGH].
 */
FlightMode getFlightMode(float flightModeValue) {

    /* Get the current flight mode from the RC. */
    FlightMode newFlightMode;
    if (flightModeValue < 1.0 / 3.0 * (RC_HIGH - RC_LOW))
        newFlightMode = FlightMode::MANUAL;
    else if (flightModeValue < 2.0 / 3.0 * (RC_HIGH - RC_LOW))
        newFlightMode = FlightMode::ALTITUDE_HOLD;
    else
        newFlightMode = FlightMode::AUTONOMOUS;

    // TODO: can this be smaller
    int MODE_DELAY = 50;
    /* Add the delay for a mode switch. */
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
 * @param   wptValue
 *          The RC value from which the WPT mode will be determined.
 * 
 * @return  WPTMode::OFF
 *          If the given value is less than RC_MID.
 * @return  WPTMode::ON
 *          If the given value is greater than RC_MID.
 */
WPTMode getWPTMode(float wptValue) {

    /* Get the current WPT mode from the RC. */
    WPTMode newWPTMode;
    if (wptValue < RC_MID)
        newWPTMode = WPTMode::OFF;
    else
        newWPTMode = WPTMode::ON;

    // TODO: can this be smaller
    int MODE_DELAY = 50;
    /* Add the delay for a mode switch. */
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
 * @param   x
 *          The value to clamp.
 * 
 * @return  RC_LOW
 *          If the given value is less than RC_LOW.
 * @return  RC_HIGH
 *          If the given value is greater than RC_HIGH.
 * @return  x
 *          Otherwise.
 */
float clamp(float x) {
    if (x < RC_LOW)
        return RC_LOW;
    if (x > RC_HIGH)
        return RC_HIGH;
    return x;
}

/**
 * Clamp value in [RC_LOW, RC_HIGH]. If the value is not in the interval
 * [RC_LOW-RC_MARGIN, RC_HIGH+RC_MARGIN], then it will be set to RC_MID.
 * 
 * @param   x
 *          The value to clamp.
 * 
 * @return  RC_MID
 *          If the given value is not in [RC_LOW-RC_MARGIN, RC_HIGH+RC_MARGIN].
 * @return  RC_LOW
 *          If the given value is in [RC_LOW-RC_MARGIN, RC_LOW].
 * @return  RC_HIGH
 *          If the given value is in [RC_HIGH, RC_HIGH+RC_MARGIN].
 * @return  x
 *          Otherwise.
 */
float clampMid(float x) {
    if (x < RC_LOW - RC_MARGIN || x > RC_HIGH + RC_MARGIN)
        return RC_MID;
    if (x < RC_LOW)
        return RC_LOW;
    if (x > RC_HIGH)
        return RC_HIGH;
    return x;
}

/**
 * Rescale to [0.0, 1.0] using deadzone at RC_LOW.
 * 
 * @param   x
 *          The value to scale, assuming this is in [RC_LOW, RC_HIGH].
 * 
 * @return  0.0
 *          If the given value is in [RC_LOW, RC_LOW+RC_DEADZONE].
 * @return  The given value scaled to [0.0, 1.0].
 *          If the given value is in [RC_LOW+RC_DEADZONE, RC_HIGH]. 
 */
float rescale(float x) {

    /* Upper edge of deadzone. */
    float low = RC_LOW + RC_DEADZONE;

    /* Scale value to [0.0, 1.0]. */
    if (x > low)
        return (x - low) / (RC_HIGH - low);

    /* Dead zone. */
    return 0.0;
}

/**
 * Rescale to [-1.0, 1.0] using deadzone at RC_MID.
 * 
 * @param   x
 *          The value to scale, assuming this is in [RC_LOW, RC_HIGH].
 * 
 * @return  0.0
 *          If the given value is in [RC_MID-RC_DEADZONE, RC_MID+RC_DEADZONE].
 * @return  The given value scaled to [-1.0, 0.0].
 *          If the given value is in [RC_LOW, RC_MID-RC_DEADZONE].
 * @return  The given value scaled to [0.0, 1.0].
 *          If the given value is in [RC_MID+RC_DEADZONE, RC_HIGH].
 */
float rescaleMid(float x) {

    /* Edges of deadzone. */
    float midlow  = RC_MID - RC_DEADZONE;
    float midhigh = RC_MID + RC_DEADZONE;

    /* Scale value to [-1.0, 1.0]. */
    if (x < midlow)
        return (x - midlow) / (midlow - RC_LOW);
    if (x > midhigh)
        return (x - midhigh) / (RC_HIGH - midhigh);

    /* Deadzone. */
    return 0.0;
}

RCInput readRC() {

    /* Get all of the RC controls. */
    float throttle        = rescale(clamp(getRCValue(THROTTLE_ADDR)));
    float pitch           = -rescaleMid(clampMid(getRCValue(PITCH_ADDR)));
    float roll            = -rescaleMid(clampMid(getRCValue(ROLL_ADDR)));
    float yaw             = -rescaleMid(clampMid(getRCValue(YAW_ADDR)));
    float tuner           = rescaleMid(clampMid(getRCValue(TUNER_ADDR)));
    float flightModeValue = rescale(clamp(getRCValue(FLIGHT_MODE_ADDR)));
    float wptValue        = rescale(clamp(getRCValue(WPT_MODE_ADDR)));

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
