// Original: BareMetal/src/main.c

/******************************************************************************
*   Main script
*   this is the first script executed when the BareMetal starts. It set's up
*   all of the components on the Zybo, starts the interrupts and runs a loop
*   that logs data.
******************************************************************************/
#include <ControllerInstances.hpp>
#include <IMU.hpp>
#include <Interrupt.hpp>
#include <Main.hpp>
#include <Motors.hpp>
#include <Platform.hpp>
#include <SharedMemoryInstances.hpp>
#include <Sonar.hpp>

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

    /* Reset PWM output. */
    outputMotorPWM(0, 0, 0, 0);

    /* Initialize the controllers and input bias. */
    initControllerInstances();

    /* Initialize the communication with the Linux core. */
    initSharedMemoryInstances();

    /* Initialize interrupt system. */
    if (initInterrupt() == false)
        return 1;

    //-------------------- MAIN EXECUTION -------------------
    while (true) {
        mainLoop();  // TODO
    }

    /* For completeness. */
    cleanupPlatform();
    return 1;
}
