#include <WPT.hpp>
#include "../../../main/src/PrivateHardwareConstants.hpp"

/** Address of the PWM period of the WPT. */
const int PERIOD_ADDR = XPAR_PWM_AXI_TRIPLE_4_0;

/** Address of the PWM duty cycle of the WPT. */
const int DUTY_CYCLE_ADDR = XPAR_PWM_AXI_TRIPLE_4_0 + 0x04;

/** PWM frequency sent to the WPT team. */
const float PWM_FREQUENCY = 90000.0;

/** Duty cycle of the PWM signal sent to the WPT team. */
const float DUTY_CYCLE = 0.50;


void disableWPT() {
    *PERIOD_ADDR     = 0;
    *DUTY_CYCLE_ADDR = 0;
}

void enableWPT() {
    *PERIOD_ADDR     = CLOCK_FREQUENCY / PWM_FREQUENCY;
    *DUTY_CYCLE_ADDR = DUTY_CYCLE * *PERIOD_ADDR;
}
