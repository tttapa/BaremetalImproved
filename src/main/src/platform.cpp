// Original: BareMetal/src/platform/platform.c
// Original: BareMetal/src/platform/eagle_ipc.c

/******************************************************************************
*   Platform source code
*
*   This file should NEVER be changed by the students.
*******************************************************************************/
#include "platform.hpp"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xiicps.h"
#include "xscugic.h"
#include <stdio.h>
#include "stdlib.h"
#include "sleep.h"


/******************************************************************************
*
* Copyright (C) 2010 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * Uncomment the following line if ps7 init source files are added in the
 * source directory for compiling example outside of SDK.
 */
/*#include "ps7_init.h"*/

#ifdef STDOUT_IS_16550
    #include "xuartns550_l.h"
    #define UART_BAUD 9600
#endif

void enableCaches() {
    #ifdef __PPC__
        Xil_ICacheEnableRegion(CACHEABLE_REGION_MASK);
        Xil_DCacheEnableRegion(CACHEABLE_REGION_MASK);
    #elif __MICROBLAZE__
        #ifdef XPAR_MICROBLAZE_USE_ICACHE
            Xil_ICacheEnable();
        #endif
        #ifdef XPAR_MICROBLAZE_USE_DCACHE
            Xil_DCacheEnable();
        #endif
    #endif
}

void disableCaches() {
    Xil_DCacheDisable();
    Xil_ICacheDisable();
}

void initUART() {
    #ifdef STDOUT_IS_16550
        XUartNs550_SetBaud(STDOUT_BASEADDR, XPAR_XUARTNS550_CLOCK_HZ, UART_BAUD);
        XUartNs550_SetLineControlReg(STDOUT_BASEADDR, XUN_LCR_8_DATA_BITS);
    #endif
    #ifdef STDOUT_IS_PS7_UART
        /* Bootrom/BSP configures PS7 UART to 115200 bps */
    #endif
}

void initXilinxPlatform() {
    /*
     * If you want to run this example outside of SDK,
     * uncomment the following line and also #include "ps7_init.h" at the top.
     * Make sure that the ps7_init.c and ps7_init.h files are included
     * along with this example source files for compilation.
     */
    /* ps7_init();*/
    enableCaches();
    initUART();
}

void cleanupPlatform() {
    disableCaches();
}




/************************************************************
*   Inter-processor communication source code
*   This file contains all functions required for the IPC.
*   This file should NEVER be changed by the students.
*   Author: r. theunis
*************************************************************/

extern u32 MMUTable;

void eagleSetupIPC(void){
	eagleSetTLBAttributes(0xFFFF0000, 0x04de2);
}

void eagleSetupClock(void) {
	uint32_t* amba_clock_control = (uint32_t*)0xF800012C;
	*amba_clock_control |= (1<<23); //Enable QSPI
	*amba_clock_control |= (1<<22); //Enable GPIO
	*amba_clock_control |= (1<<18); //Enable I2C0
	*amba_clock_control |= (1<<19); //Enable I2C1
}

void eagleDCacheFlush(void) {
	Xil_L1DCacheFlush();
	//Xil_L2CacheFlush();
}

void eagleSetTLBAttributes(u32 addr, u32 attrib) {
	u32 *ptr;
	u32 section;

	mtcp(XREG_CP15_INVAL_UTLB_UNLOCKED, 0);
	dsb();

	mtcp(XREG_CP15_INVAL_BRANCH_ARRAY, 0);
	dsb();
	eagleDCacheFlush();

	section = addr / 0x100000;
	ptr = &MMUTable + section;
	*ptr = (addr & 0xFFF00000) | attrib;
	dsb();
}

void initPlatform() {

	/* Configure inter-processor communication (only needed in AMP mode). */
	eagleSetupIPC();

	/* Set up Xilinx platform. */
	initXilinxPlatform();

	/* Setup clock control. */
	eagleSetupClock();
}



// TODO: author
/**********************************************************************************************************************
*   Other functions used in BareMetal. These are moved to platform.hpp for abstraction of the platform. in tIntegrated Integrated Circuit device driver source code
*
*   This file should NEVER be changed by the students.
*   Author: /
***********************************************************************************************************************/

void generateHeartbeat() {
    XGpio_DiscreteWrite(&axi_gpio_1, HEARTBEAT_CHANNEL, 0);
    XGpio_DiscreteWrite(&axi_gpio_1, HEARTBEAT_CHANNEL, 1);
}

void testpinHigh() {
    XGpio_DiscreteWrite(&axi_gpio_2, 1, 0x1);
}

void writeValueToLEDs(int value) {
    XGpio_DiscreteWrite(&axi_gpio_1, LED_CHANNEL, value);
}