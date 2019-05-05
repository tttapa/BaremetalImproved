// Original: BareMetal/src/RC/RC.c
/**********************************************************************************************************************
*   Radio Control source file
*   This file contains all parameters used to read input from the RC.
*   This file should normally not be changed by the students.
*   Author: w. devries, p. coppens
***********************************************************************************************************************/
#include "../include/RC.hpp"
#include "../../../main/src/HardwareConstants.hpp"
#include <xil_io.h>

//The last mode that the drone was in at the last interrupt.
int lastMode      = 0;
int lastInductive = 0;

int newModeCounter      = 0;
int newInductiveCounter = 0;

int getFlightMode(float modeValue) {

    float threshold1 = (RC::RC_HIGH - RC::RC_LOW) * 1.0 / 3.0;
    float threshold2 = (RC::RC_HIGH - RC::RC_LOW) * 2.0 / 3.0;

    // Get the current mode from RC
    int newMode;
    if (mode < threshold1)
        newMode = FlightMode::MANUAL;
    else if (mode < threshold2)
        newMode = FlightMode::ALTITUDE_HOLD_MODE;
    else
        newMode = FlightMode::AUTONOMOUS_MODE;

// TODO: can this be smaller
#define MODE_DELAY 50
    // Add the delay for a mode switch
    if (newMode != lastMode) {
        newModeCounter++;
        if (newModeCounter > MODE_DELAY) {
            newModeCounter = 0;
            lastMode       = newMode;
            return newMode;
        }
    } else {
        newModeCounter = 0;
    }
    return lastMode;
}

int getWPTMode(float inductive) {

    int newInductive;
    if (inductive < RC::RC_MID)
        newInductive = WPTMode::OFF;
    else
        newInductive = WPTMode::ON;

// TODO: can this be smaller
#define MODE_DELAY 50
    // Add the delay for a mode switch
    if (newInductive != lastInductive) {
        newInductiveCounter++;
        if (newInductiveCounter > MODE_DELAY) {
            newInductiveCounter = 0;
            lastInductive       = newInductive;
            return newInductive;
        }
    } else {
        newInductiveCounter = 0;
    }
    return lastInductive;
}

/**
 * Clamp value in [RC_LOW, RC_HIGH].
 * 
 *  @return clamped value.
 */
float clamp(float x) {
    if (x < RC::RC_LOW)
        return RC::RC_LOW;
    if (x > RC::RC_HIGH)
        return RC::RC_HIGH;
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
    if (x < RC::RC_LOW - RC::RC_MARGIN || x > RC::RC_HIGH + RC::RC_MARGIN)
        return RC::RC_MID;
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
    float low = RC::RC_LOW + RC::RC_MARGIN;

    /* Out of range. */
    if (x < low)
        return 0.0;
    if (x > RC::RC_HIGH)
        return 1.0;

    /* In range. */
    return (x - low) / (RC::RC_HIGH - low);
}

/**
 * Rescale to [-0.5, 0.5] using deadzone at RC_MID.
 * 
 *  @return rescaled value.
 */
float rescaleMid(float x) {

    /* Edges of deadzone. */
    // TODO: change this to "RC_DEADZONE_SIZE"
    float midlow  = RC::RC_MID - RC::RC_MARGIN;
    float midhigh = RC::RC_MID + RC::RC_MARGIN;

    /* Out of range. */
    if (x < RC::RC_LOW)
        return -0.5;
    if (x > RC::RC_HIGH)
        return 0.5;

    /* In range. */
    if (x < midlow)
        return (x - midlow) / (midlow - RC::RC_LOW);
    if (x > midhigh)
        return (x - midhigh) / (RC::RC_HIGH - midhigh);

    /* Deadzone. */
    return 0;
}

RCInput readRC() {

    // Get all of the rc controls
    float throttle =
        rescale(clamp((float) Xil_In32(RC::THROTTLE_ADDR) / CLOCK_FREQUENCY));
    float pitch = rescaleMid(
        clampMid((float) Xil_In32(RC::PITCH_ADDR) / CLOCK_FREQUENCY));
    float roll =
        rescaleMid(clampMid((float) Xil_In32(RC::ROLL_ADDR) / CLOCK_FREQUENCY));
    float yaw =
        rescaleMid(clampMid((float) Xil_In32(RC::YAW_ADDR) / CLOCK_FREQUENCY));
    float tuner = rescaleMid(clampMid(
        (float) Xil_In32(RC::TUNER_ADDR) /
        CLOCK_FREQUENCY));  // TODO: dead should stay mid, but margin should increase
    float mode =
        rescale(clamp((float) Xil_In32(RC::MODE_ADDR) / CLOCK_FREQUENCY));
    float inductive =
        rescale(clamp((float) Xil_In32(RC::INDUCTIVE_ADDR) / CLOCK_FREQUENCY));

    // Scale throttle to [0.00, 0.80] so that we can still make attitude adjustments when throttle
    // reaches its highest value
    float MAX_THROTTLE = 0.80;
    throttle           = throttle * MAX_THROTTLE;

    return RCInput{throttle,
                   pitch,
                   roll,
                   yaw,
                   tuner,
                   getMode(mode),
                   getInductive(inductive)};
}
