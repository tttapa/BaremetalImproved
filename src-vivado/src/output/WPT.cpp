#include "../PrivateHardwareConstants.hpp"
#include <WPT.hpp>

/** Address of the PWM period of the WPT. */
const int PERIOD_ADDR = XPAR_PWM_AXI_TRIPLE_4_0;

/** Address of the PWM duty cycle of the WPT. */
const int DUTY_CYCLE_ADDR = XPAR_PWM_AXI_TRIPLE_4_0 + 0x04;

/** PWM frequency sent to the WPT team. */
const float PWM_FREQUENCY = 500.0;

void outputWPT(float dutyCycle) {
    *PERIOD_ADDR     = CLOCK_FREQUENCY / PWM_FREQUENCY;
    *DUTY_CYCLE_ADDR = dutyCycle * *PERIOD_ADDR;
}
