#include <iostream>

/* Includes from src. */
#include <ControllerInstances.hpp>
#include <MainLoop.hpp>
#include <SharedMemoryInstances.hpp>

/* Includes from src-vivado. */
#include <output/Buzzer.hpp>
#include <output/Motors.hpp>
#include <output/WPT.hpp>
#include <platform/AxiGpio.hpp>
#include <platform/Interrupt.hpp>
#include <platform/Platform.hpp>
#include <sensors/IMU.hpp>
#include <sensors/Sonar.hpp>

/**
 * @brief   Main entry point to the Baremetal applications.
 */
int main(void) {

    std::cout << "Hello World from Baremetal" << std::endl;

    /* Initialize Xilinx platform and IPC. */
    initPlatform();

    /* Initialize interrupt system. */
    if (initInterrupt() == false)
        return 1;

    /* Initialize AXI GPIOs. */
    if (initAxiGpio() == false)
        return 1;

    /* Initialize the controllers and input bias. */
    initControllerInstances();

    /* Initialize the communication with the Linux core. */
    initSharedMemoryInstances();

    /* Initialize the sensors. AHRS will be initialized after IMU is calibrated.  */
    initSonar();
    initIMU();
    if (initIMUInterruptSystem() == false)
        return 1;

    /* Reset PWM output. */
    outputMotorPWM({0, 0, 0, 0});
    outputBuzzerPWM({0, 0, 0});
    outputWPT(0);

    //-------------------- MAIN EXECUTION -------------------
    std::cout << "Main Execution started" << std::endl;
    while (true) {
        mainLoop();  // TODO
    }

    /* For completeness. */
    cleanupPlatform();
    return 1;
}
