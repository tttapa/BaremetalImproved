#include <xparameters.h>
#pragma once

/**
 * Anonymous namespace containing frequency of measurement clock connected to
 * AXI bus.
 */
namespace {

    // TODO: is this an int or a float?
    /* Frequency of measure clock connected to AXI BUS on FCLK_CLK0. */
    const int MEASURE_FREQ XPAR_PS7_UART_1_UART_CLK_FREQ_HZ

}


/**
 * Constants used by the AXI GPIOs.
 */
namespace AXI_GPIO {

	/* Port used for LEDs in LED GPIO. */
	const int LED_CHANNEL = 1;

	/* Port used for heartbeat in LED GPIO. */
	const int HEARTBEAT_CHANNEL = 2;

	/* Port used for testpin in TESTPIN GPIO. */
	const int TESTPIN_CHANNEL = 1;

	/* Address of GPIO that LEDs are connected to. */
	const int GPIO_DEVICE_LED = XPAR_AXI_GPIO_LED_DEVICE_ID;

	/* Address of GPIO that the TESTPINs are connected to. */
	const int GPIO_DEVICE_TESTPIN = XPAR_AXI_GPIO_TESTPINS_DEVICE_ID;

}


/**
 * Addresses used by the AXI GPIOs.
 */
namespace HARDWARE_ADDRESSES {



    /* Address of the sonar : // TODO: what pin? */
    const int SONAR = XPAR_RC_1_S00_AXI_BASEADDR + 0x04;    // TODO: this was SONAR_REG


}

/**
 * Constants used by the IMU.
 */
namespace IMU {
    
    /* Amount of samples to take to determine bias. */
	const int CALIBRATION_SAMPLES = 512;
	
    /* Amount of samples to remove at the start of calibration. */
	const int INVALID_SAMPLES = 16;

    /* Number of u8s used to construct the 3 raw measurements (signed 16-bit) for the gyroscope. */
    const int GYRO_DATA_SIZE = 6;

    /* Number of u8s used to construct the 3 raw measurements (signed 16-bit) for the accelerometer. */
    const int ACCEL_DATA_SIZE = 6;

    /* Maximum measurable angular velocity in degree/s. */
    const float MAX_GYRO_VALUE = 2000.0;

    /* Maximum measurable acceleration in g. */
    const float MAX_ACCEL_VALUE = 16.0;
}


/**
 * Constants used by the interrupt system.
 */
namespace INTERRUPT_SYSTEM {

    /* Interrupt controller device ID. */
    const int INTC_DEVICE_ID = XPAR_PS7_SCUGIC_0_DEVICE_ID;

    /* ID of interrupt generated by I2C when data is being processed. */
    const int IIC_INT_VEC_ID = XPAR_XIICPS_0_INTR;

    /* ID of interrupt generated by IMU via JB4 pin. */
    const int GYR_INT_ID = XPAR_FABRIC_SYSTEM_CORE1_NIRQ_INTR;

    /* I2C serial clock frequency in hertz. */
    // TODO: inconsistent with used clock rate, see Interrupt.cpp
	const int IIC_SCLK_RATE = 100010; 

}


/**
 * Constants used by the RC.
 */
namespace RC {


    // TODO: are these pins correct?

    /* Address of the RC's throttle: PIN T14 (JD1). */
    const int THROTTLE_ADDR = XPAR_RC_0_S00_AXI_BASEADDR;     // TODO: this was RC_T

    /* Address of the RC's roll: PIN T15 (JD2). */
    const int ROLL_ADDR = XPAR_RC_0_S00_AXI_BASEADDR + 0x04;  // TODO: this was RC_Y

    /* Address of the RC's pitch: PIN P14 (JD3). */
    const int PITCH_ADDR = XPAR_RC_0_S00_AXI_BASEADDR + 0x08; // TODO: this was RC_X

    /* Address of the RC's yaw: PIN R14 (JD4). */
    const int YAW_ADDR = XPAR_RC_0_S00_AXI_BASEADDR + 0x0C;   // TODO: this was RC_Z

    /* Address of the RC's mode: PIN U15 (JD6). */
    const int MODE_ADDR = XPAR_RC_1_S00_AXI_BASEADDR;

    /* Address of the RC's inductive switch: PIN V17 (JD7). */
    const int INDUCTIVE_ADDR = XPAR_RC_1_S00_AXI_BASEADDR + 0x08; // TODO: this was RC_IND

    /* Address of the RC's tuner knob: // TODO: what pin? */
    const int TUNER_ADDR = XPAR_RC_1_S00_AXI_BASEADDR + 0x0C; // TODO: this was RC_TUNE



    // TODO: real_t?
    /* Value if RC knob/joystick is at its lowest value. */
    const float RC_LOW = 0.001109;

    /* Value if RC knob/joystick is at its highest value. */
    const float RC_HIGH = 0.001890;

    /* Center value of the interval [RC_LOW, RC_HIGH]. */
    const float RC_MID = (RC_LOW + RC_HIGH) / 2.0;

    /* Size of RC knob/joystick deadzone. */
    const float RC_MARGIN = (RC_HIGH - RC_LOW) / 40.0;

    /* Value if RC knob/joystick is not available. */
    const float RC_DEAD = 0.0;

}


namespace SONAR {

    /** Conversion factor to meters. */
    const float PWM_TO_HEIGHT = 0.005787;

    /** Size of the median filter buffer during flight. */
    const int MF_BUFFER_SIZE_SMALL = 5;

    /** 
     * Total number of measurements stored in the sonar measurement buffer. This is 
     * the maximum median filter window size. 
     */
    const int MAX_MF_LENGTH = 15;

    /** 
     * Maximum amount of subsequent iterations where the peak filter is allowed to
     * discard a measurement. 
     */
    const int MAX_JUMP_COUNT = 3;

    /** 
     * Maximum amount of meters the height can change in 1 iteration 
     * (after applying median filter). 
     */
    const float MAX_JUMP = 0.5;

}


