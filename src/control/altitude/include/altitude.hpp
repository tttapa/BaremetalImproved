struct AltitudeReference {
    float z;    // Height (m)
};

struct AltitudeState {
    float nt;   // Common motor marginal angular velocity (rad/s)
    float z;    // Height (m)
    float vz;   // Velocity (m/s)
};

struct AltitudeControlSignal {
    float ut;   // Common motor marginal signal (/)
}

// Original: BareMetal/src/control/altitude.h

/**********************************************************************************************************************
*   Altitude controller header file
*   this script contains functions used to generate inputs to the drone
*   that allow it to fly at a constant height
*   author: p. coppens
***********************************************************************************************************************/
#ifndef ALTITUDE_H
#define ALTITUDE_H

// Header Files
// ====================================================================================================================
#include <main.h>
#include <math.h>

// Constant definitions
// ====================================================================================================================
#define C2NH	116.55 		/* Constant ratio between hovering thrust c_h and hovering propeller rotation speed n_h */
#define THRUST_CLIP 0.1		/* Clamp parameter for u_thrust */

// Prototype definitions
// ====================================================================================================================
void altitude_flying(float rz);
void altitude_init();
void correct_tilt_height();
void thrust_out();


// Macros
// ====================================================================================================================

/**
 * Used to restrict a variable to a certain interval.
 *
 * parameters:
 *  	x:      the variable to restrict to an interval
 *  	lo:     the lower boundary
 *  	hi:     the higher boundary
 */
#define CLAMP_INPLACE(x, lo, hi) { \
	if((x) < ((lo))) \
		(x) = (lo); \
	if((x) > ((hi))) \
		(x) = (hi); \
}

// Global Variables
// ====================================================================================================================
/* controller inputs */
float target_z;			/* The target height of the drone, initialized by altitude_init() */
float pz;				/* The current measurement of the height after tilt correction (if implemented).
						 * See sonar.c for calculation of variable 'sonar' (=before tilt correction) */

/* Controller output */
float u_thrust;			/* The thrust with the hovering bias removed */

/* altitude output */
float alt_thrust; 		/* The value for thrust calculated by the altitude controller, equal to u_thrust + c_h (when not clamped)
						 * This value will overwrite the thrust value from the RC while in altitude and navigation mode. */

// TODO: prevent sonar from firing two measurements back to back... this messes up the KF
// int alt_ticksPassed;
// int alt_minTicksPassed;

#endif // ALTITUDE_H
