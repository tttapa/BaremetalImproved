// Original: BareMetal/src/control/buzzercodesiel.c
#include <buzzer.hpp>

int Buzzer::isBufferEmpty() {
    if (this->writeIndex == this->readIndex &&
        this->numberInstructionsLeft == 0)
        return 1;
    else
        return 0;
}

int isBufferFull() {
    if (this->writeIndex == this->readIndex && this->numberInstructionsLeft > 0)
        return 1;
    else
        return 0;
}

int incrementReadIndex() {

    // Buffer empty
    if (isBufferEmpty() == 1)
        return 0;

    // Increment
    this->readIndex++;
    this->numberInstructionsLeft--;
    if (this->readIndex >= BEEP_INSTR_QUEUE_SIZE)
        this->readIndex = 0;

    // Success
    return 1;
}

// Returns success
int incrementWriteIndex() {

    // Buffer full
    if (isBufferFull() == 1)
        return 0;

    // Increment
    this->writeIndex++;
    this->numberInstructionsLeft++;
    if (this->writeIndex >= BEEP_INSTR_QUEUE_SIZE)
        this->writeIndex = 0;

    // Success
    return 1;
}

void addConfigurationBeeps(int numberOfBeeps) {

    int i;
    for (i = 0; i < numberOfBeeps; i++) {
        if (isBufferFull() == 0) {
            beepQueue[0][this->writeIndex] = BEEP_CONFIG_DURATION;
            beepQueue[1][this->writeIndex] = BEEP_CONFIG_PERIOD;
            beepQueue[2][this->writeIndex] = BEEP_CONFIG_VOLUME;
            incrementWriteIndex();
        }
        if (isBufferFull() == 0) {
            beepQueue[0][this->writeIndex] = BEEP_CONFIG_DELAY;
            beepQueue[1][this->writeIndex] = 0;
            beepQueue[2][this->writeIndex] = 0;
            incrementWriteIndex();
        }
    }
}

void addWarningBeeps(int numberOfBeeps) {

    int i;
    for (i = 0; i < numberOfBeeps; i++) {
        if (isBufferFull() == 0) {
            beepQueue[0][this->writeIndex] = BEEP_WARNING_DURATION;
            beepQueue[1][this->writeIndex] = BEEP_WARNING_PERIOD;
            beepQueue[2][this->writeIndex] = BEEP_WARNING_VOLUME;
            incrementWriteIndex();
        }
        if (isBufferFull() == 0) {
            beepQueue[0][this->writeIndex] = BEEP_WARNING_DELAY;
            beepQueue[1][this->writeIndex] = 0;
            beepQueue[2][this->writeIndex] = 0;
            incrementWriteIndex();
        }
    }
}

void updateBuzzer() {

    // Instructions left?
    if (isBufferEmpty() == 0) {
        instructionTickCounter++;

        // Next instruction
        if (instructionTickCounter > beepQueue[0][readIndex]) {
            *sounder_period = beepQueue[1][readIndex];
            *sounder_volume = beepQueue[2][readIndex];
            incrementReadIndex();
            instructionTickCounter = 0;
        }

    } else {
        // Edge case: if last instruction to be executed is a buzzer sound, then
        //            it would play indefinitely...
        *sounder_period = 0x0000;
        *sounder_volume = 0x0000;
    }
}

void updateNavIdleBuzzer() {

    navIdleBuzzerCounter++;

    if (navIdleBuzzerCounter >= BEEP_NAVIDLE_DURATION + BEEP_NAVIDLE_DELAY) {
        if (isBufferFull() == 0) {
            beepQueue[0][writeIndex] = BEEP_NAVIDLE_DURATION;
            beepQueue[1][writeIndex] = BEEP_NAVIDLE_PERIOD;
            beepQueue[2][writeIndex] = BEEP_NAVIDLE_VOLUME;
            incrementWriteIndex();
        }
        navIdleBuzzerCounter = 0;
    }
}

// in PWM.c
void beepArmed() {}

void beepDisarmed() {}