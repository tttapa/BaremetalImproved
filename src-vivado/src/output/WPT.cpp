#include "../PrivateHardwareConstants.hpp"
#include <WPT.hpp>
#include <cstdint>

/** Address of the PWM period of the WPT. */
uint32_t * const PERIOD_ADDR = (uint32_t *) XPAR_PWM_AXI_TRIPLE_4_0;

/** Address of the PWM duty cycle of the WPT. */
uint32_t * const DUTY_CYCLE_ADDR = ((uint32_t *) XPAR_PWM_AXI_TRIPLE_4_0) + 1;

/** PWM frequency sent to the WPT team. */
const float PWM_FREQUENCY = 500.0;

void outputWPT(float dutyCycle) {
    *PERIOD_ADDR     = CLOCK_FREQUENCY / PWM_FREQUENCY;
    *DUTY_CYCLE_ADDR = dutyCycle * *PERIOD_ADDR;
}
