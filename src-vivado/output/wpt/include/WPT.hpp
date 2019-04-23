/**
 * Turn off the Wireless Power Transfer.
 */
void disableWPT() {
    *WPT::PERIOD_ADDR = 0;
    *WPT::DUTY_CYCLE_ADDR = 0;
}


/**
 * Turn on the Wireless Power Transfer.
 */
void enableWPT() {
    *WPT::PERIOD_ADDR = CLOCK_FREQUENCY / WPT::PWM_FREQUENCY;
    *WPT::DUTY_CYCLE_ADDR = WPT::DUTY_CYCLE * *WPT::PERIOD_ADDR;
}
