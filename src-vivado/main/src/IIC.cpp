
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
