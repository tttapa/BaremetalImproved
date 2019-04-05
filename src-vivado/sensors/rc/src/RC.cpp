// Original: BareMetal/src/RC/RC.c
/**********************************************************************************************************************
*   Radio Control source file
*   This file contains all parameters used to read input from the RC.
*   This file should normally not be changed by the students.
*   Author: w. devries, p. coppens
***********************************************************************************************************************/
#include "RC.hpp"
#include "../../main/HardwareConstants.hpp"


//The last mode that the drone was in at the last interrupt.
int lastMode = 0;
int lastInductive = 0;

// TODO: is this necessary
int newModeCounter = 0;
int newInductiveCounter = 0;


int getMode(float mode) {

    float threshold1 = (RC::RC_HIGH - RC::RC_LOW) * 1.0 / 3.0;
    float threshold2 = (RC::RC_HIGH - RC::RC_LOW) * 2.0 / 3.0;

	// Get the current mode from RC
	int newMode;
	if (mode < threshold1)
		newMode = MODE::MANUAL;
	else if (mode < threshold2)
		newMode = MODE::ALTITUDE_HOLD;
	else
        newMode = MODE::AUTONOMOUS; 

    // TODO: is this necessary?
    // TODO: -> maybe bad measurement rejection  
    // TODO: -> maybe preventing switching too fast?  
    #define MODE_DELAY 50
	// Add the delay for a mode switch
	if (newMode != lastMode) {
		newModeCounter++;
		if (newModeCounter > MODE_DELAY) {
			newModeCounter = 0;
			lastMode = newMode;
			return newMode;
		}
	} else {
		newModeCounter = 0;
	}
    return lastMode;
}


int getInductive(float inductive) {

	int newInductive;
	if (inductive < RC::RC_MID)
		newInductive = INDUCTIVE::OFF;
	else
		newInductive =  INDUCTIVE::ON;

    // TODO: is this necessary
    #define MODE_DELAY 50
	// Add the delay for a mode switch
	if (newInductive != last_rc_ind) {
		newInductiveCounter++;
		if (newInductiveCounter > MODE_DELAY) {
			newInductiveCounter = 0;
			lastInductive = newInductive;
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
float clamp(x) {
    if(x < RC::RC_LOW)
        return RC::RC_LOW;
    if(x > RC::RC_HIGH)
        return RC::RC_HIGH;
    return x;
}


/**
 * Clamp value in [RC_LOW, RC_HIGH]. If value is not in this interval, it will
 * be set to RC_MID.
 * 
 *  @return clamped value.
 */
float clampMid(x) {
    // TODO: why is the deadzone margin here?
    if(x < RC::RC_LOW - RC::RC_MARGIN || x > RC::RC_HIGH + RC:RC::RC_MARGIN)
        return RC::RC_MID;
    return x;
}


/**
 * Rescale to [0, 1] using deadzone at RC_LOW.
 * 
 *  @return rescaled value.
 */
float rescale(x) {

    /* Upper edge of deadzone. */
    float low = RC::RC_LOW + RC::RC_MARGIN;

    /* Out of range. */
    if(x < low)
        return 0.0;
    if(x > RC::RC_HIGH)
        return 1.0;

    /* In range. */
    return (x - low) / (RC::RC_HIGH - low);
}


/**
 * Rescale to [-0.5, 0.5] using deadzone at RC_MID.
 * 
 *  @return rescaled value.
 */
float rescaleMid(x) {

    /* Edges of deadzone. */
    float midlow = RC_MID - RC::RC_MARGIN;
    float midhigh = RC_MID + RC::RC_MARGIN;

    /* Out of range. */
    if(x < RC::RC_LOW)
        return -0.5;
    if(x > RC::RC_HIGH)
        return 0.5;

    /* In range. */
    if(x < midlow)
        return (x - midlow) / (midlow - RC::RC_LOW);
    if(x > midhigh)
        return (x - midhigh) / (RC::RC_HIGH - midhigh);

    /* Deadzone. */
    return 0;

}


RCInput readRC() {

	// Get all of the rc controls
	float thrust    = rescale(   clamp(   (float)Xil_In32(RC::THROTTLE_ADDR)/CLK_MEASURE);
	float pitch     = rescaleMid(clampMid((float)Xil_In32(RC::PITCH_ADDR)/CLK_MEASURE);
	float roll      = rescaleMid(clampMid((float)Xil_In32(RC::ROLL_ADDR)/CLK_MEASURE);
	float yaw       = rescaleMid(clampMid((float)Xil_In32(RC::YAW_ADDR)/CLK_MEASURE);

	float tuner     = rescaleMid(clampMid((float)Xil_In32(RC_TUNE)/CLK_MEASURE));

	float mode      = rescale(   clamp(   (float)Xil_In32(RC_MODE)/CLK_MEASURE);
	float inductive = rescale(   clamp(   (float)Xil_In32(RC_IND)/CLK_MEASURE));


    return RCInput {thrust, pitch, roll, yaw, tuner, getMode(mode), getInductive(inductive)};

}


