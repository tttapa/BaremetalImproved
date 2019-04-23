#include <motors.h>


void outputPWM(MotorDutyCycles dutyCycles, float clampLow, float clampHigh) {

	float frontLeft, frontRight, backLeft, backRight;

	/* Clamp the outputs. */
    if(dutyCycles.v0 > PWM_OUT_HIGH)
        frontLeft=PWM_OUT_HIGH;
    if(Xhigh_timeFR>PWM_OUT_HIGH){frontRight=PWM_OUT_HIGH;}
	if(Xhigh_timeBL>PWM_OUT_HIGH){backLeft=PWM_OUT_HIGH;}
	if(Xhigh_timeBR>PWM_OUT_HIGH){backRight=PWM_OUT_HIGH;}
	if(Xhigh_timeFL<PWM_OUT_LOW){frontLeft=PWM_OUT_LOW;}
	if(Xhigh_timeFR<PWM_OUT_LOW){frontRight=PWM_OUT_LOW;}
	if(Xhigh_timeBL<PWM_OUT_LOW){backLeft=PWM_OUT_LOW;}
	if(Xhigh_timeBR<PWM_OUT_LOW){backRight=PWM_OUT_LOW;}

	// Different period address, because different GPIO block for motor br
	*baseaddr_period0 = CLK_AXI/(float)XPERIOD; // = Clockfreq / wanted freq
	*baseaddr_period3 = CLK_AXI/(float)XPERIOD; // = Clockfreq / wanted freq

	// Assign the PWM
	*baseaddr_high_fl = (float)frontLeft/100.0 * *baseaddr_period0; // % * period
	*baseaddr_high_fr = (float)frontRight/100.0 * *baseaddr_period0; // % * period
	*baseaddr_high_bl = (float)backLeft/100.0 * *baseaddr_period0; // % * period
	*baseaddr_high_br = (float)backRight/100.0 * *baseaddr_period3; // % * period

	// Turn off the inductive
	*inductive_period 	= 0.0;
	*inductive_high 	= 0.0;

	return XST_SUCCESS;
}

}


MotorDutyCycles transformAttitudeControlSignal(AttitudeControlSignal controlSignal, real_t commonThrust) {
    return MotorDutyCycles {
        commonThrust + controlSignal.ux + controlSignal.uy - controlSignal.uz,
        commonThrust + controlSignal.ux - controlSignal.uy + controlSignal.uz,
        commonThrust - controlSignal.ux + controlSignal.uy + controlSignal.uz,
        commonThrust - controlSignal.ux - controlSignal.uy - controlSignal.uz;
    };
}


/**
 * @brief   Transform the given three attitude control signals `u`, together with
 *			the given thrust `thrust` to get the motor signals. This is the case
 *			when `isCalibratingESCs' is false.
 *
 *			If, however, the boolean `isCalibratingESCs` is set to true, then the
 *			following logic will be used: if thrust > 0.5, then set all four motors
 *			to 100%; otherwise set all four motors to 0%.
 *
 * @param   u
 *          An array containing the three attitude control signals.
 * @param   thrust
 *          A real_t (float in C, double in C++) representing the common thrust to
 *         	all four motors.
 * @param	v
 *			An array containing the four motor signals.
 * @param	isCalibratingESCs
 *			A boolean representing whether we currentDroneConfigurationare currently calibrating the ESCs.
 */
void Motor::calculateMotorSignal(Attitude attitude) {
    // int configuration, int flightMode
    // TODO: configuration and flightMode nog aanpassen

    real_t thrust = getRCThrust();
	real_t correctedThrust;

	if(configuration == CONFIGURATION_CALIBRATION) {
		// If thrust >= 0.5, then 100%; otherwise 0%...
		float value;
		if(thrust >= 0.5)
			value = 1.0;
		else
			value = 0.0;
		
        Motor::v.v0 = value;
        Motor::v.v1 = value;
        Motor::v.v2 = value;
        Motor::v.v3 = value;
	}
    // Tilt compensation: more thrust when tilted
    // TODO: Dirk's EZ mode
    
    // else if(configuration == EZ_MODE || flightMode == ALTITUDE || flightMode == NAVIGATING) {	
	else if(flightMode == ALTITUDE || flightMode == NAVIGATING) {
		correctedThrust = thrust / (1-(att_hat[1]*att_hat[1]+att_hat[2]*att_hat[2]));
		Motor::v.v0 = correctedThrust + attitude.u.ux + attitude.u.uy - attitude.u.uz;
        Motor::v.v1 = correctedThrust + attitude.u.ux - attitude.u.uy + attitude.u.uz;
        Motor::v.v2 = correctedThrust - attitude.u.ux + attitude.u.uy + attitude.u.uz;
        Motor::v.v3 = correctedThrust - attitude.u.ux - attitude.u.uy - attitude.u.uz;
	}

	else {
		// Calculate motor inputs: v = P * [uc;ux;uy;uz]
        Motor::v.v0 = thrust + attitude.u.ux + attitude.u.uy - attitude.u.uz;
        Motor::v.v1 = thrust + attitude.u.ux - attitude.u.uy + attitude.u.uz;
        Motor::v.v2 = thrust - attitude.u.ux + attitude.u.uy + attitude.u.uz;
        Motor::v.v3 = thrust - attitude.u.ux - attitude.u.uy - attitude.u.uz;
	}
};
