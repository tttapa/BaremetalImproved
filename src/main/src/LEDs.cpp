#include "AxiGpio.hpp"


void updateLEDs(bool isInterruptRunning,
                bool isControllerArmed,
                bool isAutonomous,
                bool isWPTActive)
{

    int ledOutput = 0x0;

    /* LED 0 is lit when the interrupts are running too slowly. */
    if (isInterruptRunning)
        ledOutput += 0x1;

    /* LED 1 is lit when the controller is armed. */
    if(isControllerArmed)
        ledOutput += 0x2;

    /* LED 2 is lit when the drone is in autonomous mode. */
    if(isAutonomous)
        ledOutput += 0x4;

    /* LED 3 is lit when wireless power transfer is active. */
    if(isWPTActive)
        ledOutput += 0x8;

    /* Write value to LEDs. */
    writeValueToLEDs(ledOutput);
}