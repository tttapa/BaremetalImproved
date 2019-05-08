#include <Buzzer.hpp>
#include <xparameters.h>

/** Address to set the period of the buzzer. */
const int PERIOD_ADDR = XPAR_PWM_AXI_TRIPLE_5_0;

/** Address to set the volume of the buzzer. */
const int VOLUME_ADDR = XPAR_PWM_AXI_TRIPLE_5_0 + 0x04;

// TODO: make this period in [0,1] volume in [0,1]
void outputBuzzerPWM(int period, int volume) {
    *PERIOD_ADDR = period;
    *VOLUME_ADDR = volume;
}