// Original: BareMetal/src/control/timers.h
#include <Time.hpp>

const int BEEP_INSTR_QUEUE_SIZE = 30;

const int BEEP_CONFIG_DURATION = TICKS_PER_SECOND * 0.25;
const int BEEP_CONFIG_PERIOD = 0x35000;
const int BEEP_CONFIG_VOLUME = 0x32000;
const int BEEP_CONFIG_DELAY = TICKS_PER_SECOND * 0.20;

const int BEEP_WARNING_DURATION = TICKS_PER_SECOND * 0.12;
const int BEEP_WARNING_PERIOD = 0x21000;
const int BEEP_WARNING_VOLUME = 0x20000;
const int BEEP_WARNING_DELAY = TICKS_PER_SECOND * 0.06;

const int BEEP_NAVIDLE_DURATION = TICKS_PER_SECOND * 0.5;
const int BEEP_NAVIDLE_PERIOD = 0x25000;
const int BEEP_NAVIDLE_VOLUMT = 0x23000;
const int BEEP_NAVIDLE_DELAY = TICKS_PER_SECOND * 0.5;


class Buzzer {

  private:
    int writeIndex;
    int readIndex;
    int numberInstructionsLeft;

  public:
    int isBufferEmpty();
    int isBufferFull();
    int incrementReadIndex();
    int incrementWriteIndex();
    void addConfigurationBeeps(int numberOfBeeps);
    void addWarningBeeps(int numberOfBeeps);
    void updateBuzzer();
    void updateNavIdleBuzzer();
    void beepArmed() {}
    void beepDisarmed() {}

}
