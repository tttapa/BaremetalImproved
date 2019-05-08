#include <Buzzer.hpp>
#include <xparameters.h>

/** Address to set the period of the buzzer. */
const int PERIOD_ADDR = XPAR_PWM_AXI_TRIPLE_5_0;

/** Address to set the volume of the buzzer. */
const int VOLUME_ADDR = XPAR_PWM_AXI_TRIPLE_5_0 + 0x04;

void outputBuzzerPWM(BuzzerInstruction instruction) {
    *PERIOD_ADDR = instruction.period;
    *VOLUME_ADDR = instruction.volume;
}