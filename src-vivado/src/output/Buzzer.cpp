#include <output/Buzzer.hpp>

/* Includes from Xilinx. */
#include <xparameters.h>

/** Address to set the period of the buzzer. */
uint32_t *const PERIOD_ADDR = (uint32_t *) XPAR_PWM_AXI_TRIPLE_5_0;

/** Address to set the volume of the buzzer. */
uint32_t *const VOLUME_ADDR = ((uint32_t *) XPAR_PWM_AXI_TRIPLE_5_0) + 1;

void outputBuzzerPWM(BuzzerInstruction instruction) {
    *PERIOD_ADDR = instruction.period;
    *VOLUME_ADDR = instruction.volume;
}

void outputBuzzerPWM(float duration, int period, int volume) {
    outputBuzzerPWM({duration, period, volume});
}
