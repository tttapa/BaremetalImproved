#include <output/Motors.hpp>
#include <platform/Interrupt.hpp>
#include <platform/Platform.hpp>
#include <sensors/IMU.hpp>
#include <sensors/Sonar.hpp>

#include <ControllerInstances.hpp>
#include <MainLoop.hpp>
#include <SharedMemoryInstances.hpp>

/**
 * @brief   Main entry point to the Baremetal applications.
 */
int main(void) {

    /* Initialize Xilinx platform and IPC. */
    initPlatform();

    /* Initialize the sensors. AHRS will be initialized after IMU is calibrated.  */
    initSonar();
    initIMU();

    /* Reset PWM output. */
    outputMotorPWM({0, 0, 0, 0});

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
