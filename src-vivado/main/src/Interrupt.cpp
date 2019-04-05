// Original: BareMetal/src/intc/intc.c

/******************************************************************
*   Interrupt control source file
*   This file contains all interrupt related methods and variables.
*   This file should NEVER be changed by the students.
*   Author: w. devries
*******************************************************************/
#include "../include/Interrupt.hpp"
#include "IIC.hpp"
#include "HardwareConstants.hpp"
#include "../../sensors/imu/src/LSM9DS1_Registers.h"
#include "../../../src/main/include/MainInterrupt.hpp"	/* update() is called at 238 Hz. */

#include <xiicps.h>
#include <xscugic.h>
#include <iostream>


/* Instance of the interrupt controller. */
static XScuGic InterruptController;

/* Instance of the IIC driver. */
static XIicPs Iic0;


/**
 *
 * This function is the handler which is used when the Gyro generates an interrupt.
 * It does this at 238Hz. This method then updates the EAGLE FSM.
 *
 * @param InstancePtr is a pointer to the XIicPs instance.
 */
void int_gyr(void *InstancePtr) {
	/* update the FSM */
    std::cout << "update" << std::endl;
	// update();
}


/**
 *
 * This function setups the interrupt system such that interrupts can occur.
 * This function is application specific since the actual
 * system may or may not have an interrupt controller.
 *
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
int setupInterruptSystem() {

	/* Initialize the interrupt controller driver so that it is ready to use. */
	XScuGic_Config *IntcConfig;

	/* Does nothing, is still there for compatibility */
	Xil_ExceptionInit();

	/* Find the Zynq itself (INTC_DEVICE_ID == 0, which I assume is the id of the Zynq as it is added first) */
	IntcConfig = XScuGic_LookupConfig(INTERRUPT_SYSTEM::INTC_DEVICE_ID);
	if (NULL == IntcConfig)
		return XST_FAILURE;

	/* Build the interrupt controller interrupt handler */
	int status = XScuGic_CfgInitialize(&InterruptController, IntcConfig, IntcConfig->CpuBaseAddress);
	if (status != XST_SUCCESS)
		return XST_FAILURE;

	/* Connect the interrupt controller interrupt handler to the hardware interrupt handling logic in the processor. */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, &InterruptController);

	/* Enable interrupts in the processor. */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}


/**
 *
 * This function setups the interrupt system such that interrupts can occur
 * for the IIC.  This function is application specific since the actual
 * system may or may not have an interrupt controller.  The IIC could be
 * directly connected to a processor without an interrupt controller.  The
 * user should modify this function to fit the application.
 *
 * @param IicPsPtr contains a pointer to the instance of the Iic
 *        which is going to be connected to the interrupt controller.
 *
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
int setupIICInterruptSystem() {

	/* Initialize the interrupt controller driver so that it is ready to use. */
	XScuGic_Config *IntcConfig;

	/* Find the Zynq itself (INTC_DEVICE_ID == 0, which I assume is the id of the Zynq as it is added first). */
	IntcConfig = XScuGic_LookupConfig(INTERRUPT_SYSTEM::INTC_DEVICE_ID);
	if (NULL == IntcConfig)
		return XST_FAILURE;

	/*
	 * Connect and enable the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	int status = XScuGic_Connect(&InterruptController, INTERRUPT_SYSTEM::IIC_INT_VEC_ID,(Xil_InterruptHandler)XIicPs_MasterInterruptHandler,	(void *)&Iic0);
	if (status != XST_SUCCESS) {
		xil_printf("IIC_interrupt failed to connect\r\n");
		return status;
	}
	XScuGic_Enable(&InterruptController, INTERRUPT_SYSTEM::IIC_INT_VEC_ID);

	return XST_SUCCESS;
}


/**
 * This function setups the interrupt system such that interrupts can occur
 * for the IMU.  This function is application specific since the actual
 * system may or may not have an interrupt controller.
 *
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
int setupIMUInterruptSystem() {

	/* Initialize the interrupt controller driver so that it is ready to use. */
	XScuGic_Config *IntcConfig;

	/* Find the Zynq itself (INTC_DEVICE_ID == 0, which I assume is the id of the Zynq as it is added first). */
	IntcConfig = XScuGic_LookupConfig(INTERRUPT_SYSTEM::INTC_DEVICE_ID);
	if (NULL == IntcConfig)
		return XST_FAILURE;

	/**
	 * Connect and enable fast interrupt for the Gyr data ready interrupt.
	 * This interrupt is triggered by an external pin called Int_gyr_cpu0
	 * (or Core0_nIRQ in block diagram) which is JB4.
	 */
	int status = XScuGic_Connect(&InterruptController, INTERRUPT_SYSTEM::GYR_INT_ID, (Xil_InterruptHandler)int_gyr, (void *)&InterruptController);
	if (status != XST_SUCCESS)
	{
	 	xil_printf("Int_Gyr_data_ready failed to connect\r\n");
		return XST_FAILURE;
	}
	XScuGic_SetPriorityTriggerType(&InterruptController, INTERRUPT_SYSTEM::GYR_INT_ID, 32, 0x03);
	XScuGic_Enable(&InterruptController, INTERRUPT_SYSTEM::GYR_INT_ID);

	return XST_SUCCESS;
}

// TODO: THIS DATA IS NEVER USED! IS THIS NECESSARY?
/**
 * This function is the handler which performs processing to handle data events
 * from the IIC.  It is called from an interrupt context such that the amount
 * of processing performed should be minimized.
 *
 * @param CallBackRef contains a callback reference from the driver, in
 *        this case it is the instance pointer for the IIC driver.
 * @param Event contains the specific kind of event that has occurred.
 */
// TODO: handler does nothing
/*
void handler(void *CallBackRef, u32 Event) {

	// The following counters are used to determine when the entire buffer has been sent and received.
	// TODO: these were declared like this in the header file... what should they be now?
	volatile u32 SendComplete;
	volatile u32 RecvComplete;
	volatile u32 TotalErrorCount;

	// All of the data transfer has been finished.
	if (0 != (Event & XIICPS_EVENT_COMPLETE_RECV))
		RecvComplete = TRUE;
	else if (0 != (Event & XIICPS_EVENT_COMPLETE_SEND))
		SendComplete = TRUE;
	else if (0 == (Event & XIICPS_EVENT_SLAVE_RDY))
		// If it is other interrupt but not slave ready interrupt, it is an error.
		TotalErrorCount++;

}
*/




/**********************************************************************************************************************
*   Integrated Integrated Circuit device driver source code
*   This file contains all functions required to use the IIC.
*   This file should NEVER be changed by the students.
*   Author: w. devries
***********************************************************************************************************************/

/**
 * Initializes the IIC driver by looking up the configuration in the config
 * table and then initializing it. Also sets the IIC serial clock rate.
 */
unsigned char iicConfig(unsigned int DeviceIdPS, XIicPs* iic_ptr) {
	xil_printf("start IIC communication\r\n");

	/* Look up the configuration in the config table. */
	XIicPs_Config* Config = XIicPs_LookupConfig(DeviceIdPS);
	if(NULL == Config) {
		xil_printf("IIC lookup FAILED \r\n");
		return XST_FAILURE;
	}

	/* Initialise the IIC driver configuration. */
	int status;
	status = XIicPs_CfgInitialize(iic_ptr, Config, Config->BaseAddress);
	if(status != XST_SUCCESS) {
		xil_printf("IIC config FAILED \r\n ");
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built TRUEly.
	 */
	status = XIicPs_SelfTest(iic_ptr);
	if (status != XST_SUCCESS) {
		xil_printf("IIC selftest FAILED \r\n");
		return XST_FAILURE;
	}

	/*
	* Connect the IIC (iic_ptr) to the interrupt subsystem such that interrupts can
	* occur.  This function is application specific.
	*
	* iic_ptr (does this by accessing iic_ptr's definition as a global variable in main.h r55)
	*/
	status = setupIICInterruptSystem();
	if (status != XST_SUCCESS) {
		xil_printf("setup interrupt system failed \r\n");
		return XST_FAILURE;
	}

	/*
	 * Setup the handlers for the IIC that will be called from the
	 * interrupt context when data has been sent and received, specify a
	 * pointer to the IIC driver instance as the callback reference so
	 * the handlers are able to access the instance data.
	 */
	// TODO: handler does nothing
	// XIicPs_SetStatusHandler(iic_ptr, (void *) iic_ptr, handler);


	// Set the IIC serial clock rate.
	status = XIicPs_SetSClk(&Iic0, 400000); // TODO: should use INTERRUPT_SYSTEM::IIC_SCLK_RATE
	if (status != XST_SUCCESS) {
			xil_printf("IIC setClock FAILED \r\n");
			return XST_FAILURE;
	}

	// Print the speed of the IIC
	u32 speed = XIicPs_GetSClk(iic_ptr);
	xil_printf("IIC speed %d\r\n",speed);

	// Finish configuration
	xil_printf("I2c configured\r\n");
	return XST_SUCCESS;
}



bool initInterrupt() {

	/* Set up interrupt system. Return false if not successful. */
	if(setupInterruptSystem() != XST_SUCCESS) {
		xil_printf("setup interrupt system failed \r\n");
		return false;
	}

	/* Initialize Integrated Integrated Circuits (IIC). */
	iicConfig(XPAR_XIICPS_0_DEVICE_ID, &Iic0);

	/* Initialize IMU interrupt. */
	if(setupIMUInterruptSystem() != XST_SUCCESS) {
		xil_printf("setup IMU interrupt system failed \r\n");
		return false;
	}

	/* Setup successul. */
	return true;
}




void iicWriteToReg(u8 register_addr, u8 u8Data, int device) {
	// Wait for the I2C to end the last operation
	while(XIicPs_BusIsBusy(&Iic0));
	//xil_printf("waiting for bus (writing task)\n");

	// Get the device address
	u16 device_addr;
	if(device)
		device_addr = LSM9DS1_GX_ADDR;
	else
		device_addr = LSM9DS1_M_ADDR;

	//Set up the required bits (2 bytes)
	u8 buf[] = { register_addr, u8Data };

	// Execute read procedure
	int status = XIicPs_MasterSendPolled(&Iic0, buf, 2, device_addr);
	if(status != XST_SUCCESS)
		xil_printf("error in master send polled\r\n");

	//xil_printf("done writing data 0x%x to: 0x%x \n", u8Data, register_addr);
}

void iicReadReg(u8* recv_buffer, u8 register_addr, int device, int size) {
	//while(XIicPs_BusIsBusy(&Iic0))
	//	xil_printf("waiting for bus (reading task)\n");
	u16 device_addr;
	if(device)
		device_addr = LSM9DS1_GX_ADDR;
	else
		device_addr = LSM9DS1_M_ADDR;
	u8 buf[] = {register_addr};

	// Check if within register range
	if(register_addr < 0x05 || register_addr > 0x37)
		xil_printf("ERROR: Cannot register address, 0x%x, out of bounds\r\n", register_addr);

	int status;
	status = XIicPs_MasterSendPolled(&Iic0, buf, 1, device_addr);
	if(status != XST_SUCCESS)
		xil_printf("Send failed %d\r\n", status);

	status = XIicPs_MasterRecvPolled(&Iic0, recv_buffer, size, device_addr);
	if(status != XST_SUCCESS)
		xil_printf("Receive failed %d\r\n", status);
}
