#include "IIC.hpp"


/* Instance of the IIC driver. */
static XIicPs Iic0;

/* I2C serial clock frequency in Hertz. */
const int IIC_SCLK_RATE = 400e3;


/**
 *
 * This function setups the interrupt system such that interrupts can occur
 * for the IIC.  This function is application specific since the actual
 * system may or may not have an interrupt controller.  The IIC could be
 * directly connected to a processor without an interrupt controller.  The
 * user should modify this function to fit the application.
 *
 * @param 	IicPsPtr
 * 			A pointer to the instance of the Iic which is going to be connected
 * 			to the interrupt controller.
 *
 * @return 	XST_SUCCESS
 * 			If successful.
 * @return	XST_FAILURE
 * 			Otherwise.
 */
int setupIICInterruptSystem() {

    /* Initialize the interrupt controller driver so that it is ready to use. */
    XScuGic_Config *IntcConfig;

    /* Find the Zynq itself (INTC_DEVICE_ID == 0, which I assume is the id of
	   the Zynq as it is added first). */
    IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    if (NULL == IntcConfig)
        return XST_FAILURE;

    /* Connect and enable the device driver handler that will be called when an
	   interrupt for the device occurs, the handler defined above performs the
	   specific interrupt processing for the device. */
    int status = XScuGic_Connect(
        &InterruptController, IIC_INT_VEC_ID,
        (Xil_InterruptHandler) XIicPs_MasterInterruptHandler, (void *) &Iic0);
    if (status != XST_SUCCESS) {
        xil_printf("IIC_interrupt failed to connect\r\n");
        return status;
    }
    XScuGic_Enable(&InterruptController, IIC_INT_VEC_ID);

    return XST_SUCCESS;
}


/**
 * This function is the handler which performs processing to handle data events
 * from the IIC.  It is called from an interrupt context such that the amount
 * of processing performed should be minimized.
 *
 * @param CallBackRef contains a callback reference from the driver, in
 *        this case it is the instance pointer for the IIC driver.
 * @param Event contains the specific kind of event that has occurred.
 */
void handler(void *CallBackRef, u32 Event) {

	(void)CallBackRef;
	(void)Event;
	/*

	// The following counters are used to determine when the entire buffer has been sent and received.
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
	*/

}


/**
 * Initializes the IIC driver by looking up the configuration in the config
 * table and then initializing it. Also sets the IIC serial clock rate.
 * 
 * @return	XST_SUCCESS
 * 			If successful.
 * @return	XST_FAILURE
 * 			Otherwise.
 */
unsigned char iicConfig(unsigned int DeviceIdPS, XIicPs *iic_ptr) {
    xil_printf("start IIC communication\r\n");

    /* Look up the configuration in the config table. */
    XIicPs_Config *Config = XIicPs_LookupConfig(DeviceIdPS);
    if (NULL == Config) {
        xil_printf("IIC lookup FAILED \r\n");
        return XST_FAILURE;
    }

    /* Initialise the IIC driver configuration. */
    int status;
    status = XIicPs_CfgInitialize(iic_ptr, Config, Config->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("IIC config FAILED \r\n ");
        return XST_FAILURE;
    }

    /* Perform a self-test to ensure that the hardware was built TRUEly. */
    status = XIicPs_SelfTest(iic_ptr);
    if (status != XST_SUCCESS) {
        xil_printf("IIC selftest FAILED \r\n");
        return XST_FAILURE;
    }

    /* Connect the IIC (iic_ptr) to the interrupt subsystem such that interrupts
	   can occur.  This function is application specific. iic_ptr does this by
	   accessing iic_ptr's definition as a global variable in main.h r55). */
    status = setupIICInterruptSystem();
    if (status != XST_SUCCESS) {
        xil_printf("setup interrupt system failed \r\n");
        return XST_FAILURE;
    }

    /* Setup the handlers for the IIC that will be called from the interrupt
	   context when data has been sent and received, specify a pointer to the
	   IIC driver instance as the callback reference so the handlers are able to
	   access the instance data. */
    XIicPs_SetStatusHandler(iic_ptr, (void *) iic_ptr, handler);

    // Set the IIC serial clock rate.
    status = XIicPs_SetSClk(&Iic0, IIC_SCLK_RATE);
    if (status != XST_SUCCESS) {
        xil_printf("IIC setClock FAILED \r\n");
        return XST_FAILURE;
    }

    // Print the speed of the IIC
    u32 speed = XIicPs_GetSClk(iic_ptr);
    xil_printf("IIC speed %d\r\n", speed);

    // Finish configuration
    xil_printf("I2c configured\r\n");
    return XST_SUCCESS;
}

void initIIC() {
    /* Initialize Inter-Integrated Circuits (IIC). */
    iicConfig(XPAR_XIICPS_0_DEVICE_ID, &Iic0);
}


void iicWriteToReg(u8 register_addr, u8 u8Data, u16 deviceAddr) {
    // Wait for the I2C to end the last operation
    while (XIicPs_BusIsBusy(&Iic0))
        ;
    //xil_printf("waiting for bus (writing task)\n");

    //Set up the required bits (2 bytes)
    u8 buf[] = {register_addr, u8Data};

    // Execute read procedure
    int status = XIicPs_MasterSendPolled(&Iic0, buf, 2, deviceAddr);
    if (status != XST_SUCCESS)
        xil_printf("error in master send polled\r\n");
    //xil_printf("done writing data 0x%x to: 0x%x \n", u8Data, register_addr);
}

void iicReadReg(u8 *recv_buffer, u8 register_addr, u16 deviceAddr, int size) {
    //while(XIicPs_BusIsBusy(&Iic0))
    //	xil_printf("waiting for bus (reading task)\n");
    u8 buf[] = {register_addr};

    // Check if within register range
    if (register_addr < 0x05 || register_addr > 0x37)
        xil_printf("ERROR: Cannot register address, 0x%x, out of bounds\r\n",
                   register_addr);

    int status;
    status = XIicPs_MasterSendPolled(&Iic0, buf, 1, deviceAddr);
    if (status != XST_SUCCESS)
        xil_printf("Send failed %d\r\n", status);

    status = XIicPs_MasterRecvPolled(&Iic0, recv_buffer, size, deviceAddr);
    if (status != XST_SUCCESS)
        xil_printf("Receive failed %d\r\n", status);
}
