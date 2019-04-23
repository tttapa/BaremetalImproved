#include "../include/Buzzer.hpp"
#include "../../../main/src/HardwareConstants.hpp"

// TODO: make this period in [0,1] volume in [0,1]
void outputBuzzerPWM(int period, int volume) {
    *BUZZER::PERIOD_ADDR = period;
    *BUZZER::VOLUME_ADDR = volume;
}