// Original: BareMetal/src/platform/platform.h
// Original: BareMetal/src/platform/platform_config.h
// Original: BareMetal/src/platform/eagle_ipc.h
// Original: BareMetal/src/comm/iic.h

#pragma once

// TODO: the comment blocks about the files are wrong
// TODO: description, author
/******************************************************************************
 *   Platform header file
 *
 *   This file should NEVER be changed by the students.
 ******************************************************************************/

/** Initialize Xilinx platform and IPC. */
void initPlatform();

/** Clean up Xilinx platform. */
void cleanupPlatform();

/******************************************************************************
 *
 * Copyright (C) 2008 - 2014 Xilinx, Inc.  All rights reserved.
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

// Constant definitions (platform configuration)
// =============================================================================
#define STDOUT_IS_PS7_UART
#define UART_DEVICE_ID 0

/*******************************************************************************
 *   Inter-processor communication header file
 *   This file contains all functions required for the IPC.
 *   This file should NEVER be changed by the students.
 *   Author: r. theunis
 ******************************************************************************/

/*******************************************************************************
 *   Other functions used in BareMetal.
 *   These are moved to platform.hpp for abstraction of the platform.
 *   Inter-Integrated Circuit device driver source code.
 *
 *   This file should NEVER be changed by the students.
 *   Author: /
 *******************************************************************************/

/**
 * Generate a heartbeat by writing 0, then 1 to the heartbeat channel.
 */
void generateHeartbeat();

/**
 * Write 1 to the test pin to probe the length of an interrupt.
 */
void testpinHigh();

/**
 * Write the given value to the four LEDs on the Zybo.
 *
 * @param   value
 *          A four bit value that will be written to the four LEDs respectively.
 */
void writeValueToLEDs(int value);