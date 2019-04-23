// Original: BareMetal/src/main.c

/******************************************************************************
*   Main script
*   this is the first script executed when the BareMetal starts. It set's up
*   all of the components on the Zybo, starts the interrupts and runs a loop
*   that logs data.
******************************************************************************/
#include "../../../src-vivado/main/include/Platform.hpp"
#include "../../../src-vivado/main/include/Interrupt.hpp"
#include "../../../src-vivado/sensors/imu/include/IMU.hpp"
#include "../../../src-vivado/sensors/sonar/include/Sonar.hpp"
#include "../../init/include/Init.hpp"


/**
 * Entry point to BareMetal program.
 *
 * When booting in AMP mode with both Linux and BareMetal present.
 * BareMetal starts after Linux runs the command: ./rwmem 0xfffffff0 0x18000000 &
 * 
 * @return Exit code 1 if initialization fails, otherwise the code will run forever.
 */
int main(void) {

	/* Initialize Xilinx platform and IPC. */
	initPlatform();

	/* Initialize the sensors. AHRS will be initialized after IMU is calibrated.  */
	initSonar();
	initIMU();
	initInputBias();

	/* Initialize communication with other core. */
	initCommunication();

	/* Reset PWM output. */
	pwmOutput(0,0,0,0);

	/* Initialize files in src. */
	createControllers(attitudeController, altitudeController, positionController);

	/* Initialize interrupt system. */
	if(initInterrupt() == false)
		return 1;


	//-------------------- MAIN EXECUTION -------------------
	while (true) {

        // Wait for next interrupt...

	}

    /* For completeness. */
	cleanup_platform();
	return 1;
}

