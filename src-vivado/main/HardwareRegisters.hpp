// TODO: WHICH XILINX HEADER DOES THIS NEED TO INCLUDE?
#include <xparameters.h>

// TODO: author

// IP-hardware definitions
#define RC_T (XPAR_RC_0_S00_AXI_BASEADDR)           ///< throttle  PIN T14 (JD1)
#define RC_Y (XPAR_RC_0_S00_AXI_BASEADDR + 0x04)    ///< roll      PIN T15 (JD2)
#define RC_X (XPAR_RC_0_S00_AXI_BASEADDR + 0x08)    ///< pitch     PIN P14 (JD3)
#define RC_Z (XPAR_RC_0_S00_AXI_BASEADDR + 0x0C)    ///< yaw       PIN R14 (JD4)
#define RC_MODE (XPAR_RC_1_S00_AXI_BASEADDR)        ///< mode      PIN U15 (JD6)
#define RC_IND (XPAR_RC_1_S00_AXI_BASEADDR + 0x08)  ///< inductive PIN V17 (JD7)
#define RC_TUNE (XPAR_RC_1_S00_AXI_BASEADDR + 0x0C) ///< tuner
#define SONAR_REG (XPAR_RC_1_S00_AXI_BASEADDR + 0x04) ///< sonar