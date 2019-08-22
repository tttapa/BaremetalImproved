#include <iostream>
#include <platform/Interrupt.hpp>

/* Includes from src: updateFSM() is called when IMU updates. */
#include <MainInterrupt.hpp>

/* Includes from src-vivado. */
#include "IIC.hpp"

/* Includes from Xilinx. */
#include <xiicps.h>
#include <xparameters.h>
#include <xscugic.h>

#include <iostream>

#pragma region Constants
/* Interrupt controller device ID. */
const int INTC_DEVICE_ID = XPAR_PS7_SCUGIC_0_DEVICE_ID;

/* ID of interrupt generated by I2C when data is being processed. */
const int IIC_INT_VEC_ID = XPAR_XIICPS_0_INTR;

#pragma endregion

/* Instance of the interrupt controller. */
static XScuGic InterruptController;


/**
 *
 * This function setups the interrupt system such that interrupts can occur.
 * This function is application specific since the actual
 * system may or may not have an interrupt controller.
 *
 * @return 	XST_SUCCESS
 * 			If successful.
 * @return	XST_FAILURE
 * 			Otherwise.
 */
int setupInterruptSystem() {

    /* Initialize the interrupt controller driver so that it is ready to use. */
    XScuGic_Config *IntcConfig;

    /* Does nothing, is still there for compatibility */
    Xil_ExceptionInit();

    /* Find the Zynq itself (INTC_DEVICE_ID == 0, which I assume is the id of
	   the Zynq as it is added first) */
    IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    if (NULL == IntcConfig)
        return XST_FAILURE;

    /* Build the interrupt controller interrupt handler */
    int status = XScuGic_CfgInitialize(&InterruptController, IntcConfig,
                                       IntcConfig->CpuBaseAddress);
    if (status != XST_SUCCESS)
        return XST_FAILURE;

    /* Connect the interrupt controller interrupt handler to the hardware
	   interrupt handling logic in the processor. */
    Xil_ExceptionRegisterHandler(
        XIL_EXCEPTION_ID_IRQ_INT,
        (Xil_ExceptionHandler) XScuGic_InterruptHandler, &InterruptController);

    /* Enable interrupts in the processor. */
    Xil_ExceptionEnable();

    return XST_SUCCESS;
}


/**********************************************************************************************************************
*   Integrated Integrated Circuit device driver source code
*   This file contains all functions required to use the IIC.
*   This file should NEVER be changed by the students.
*   Author: w. devries
***********************************************************************************************************************/


bool initInterrupt() {

    /* Set up interrupt system. Return false if not successful. */
    if (setupInterruptSystem() != XST_SUCCESS) {
        xil_printf("setup interrupt system failed \r\n");
        return false;
    }

    /* Setup successful. */
    return true;
}
