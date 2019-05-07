// Original: BareMetal/src/control/buzzercodesiel.c
#include "../../../src-vivado/output/buzzer/include/Buzzer.hpp"
#include <BuzzerManager.hpp>

void BuzzerManager::addArmedBeep() {
    if (getNumInstructionsUntilFull() >= 2) {
        /* First beep. */
        this->beepQueue[this->writeIndex].duration = ARMED_DURATION1;
        this->beepQueue[this->writeIndex].period   = ARMED_PERIOD1;
        this->beepQueue[this->writeIndex].volume   = ARMED_VOLUME1;
        incrementWriteIndex();

        /* Second beep. */
        this->beepQueue[this->writeIndex].duration = ARMED_DURATION2;
        this->beepQueue[this->writeIndex].period   = ARMED_PERIOD2;
        this->beepQueue[this->writeIndex].volume   = ARMED_VOLUME2;
        incrementWriteIndex();
    }
    if (!isQueueFull()) {
        /* Delay. */
        this->beepQueue[this->writeIndex].duration = ARMED_DELAY;
        this->beepQueue[this->writeIndex].period   = 0;
        this->beepQueue[this->writeIndex].volume   = 0;
        incrementWriteIndex();
    }
}

void BuzzerManager::addDisarmedBeep() {
    if (getNumInstructionsUntilFull() >= 2) {
        /* First beep. */
        this->beepQueue[this->writeIndex].duration = DISARMED_DURATION1;
        this->beepQueue[this->writeIndex].period   = DISARMED_PERIOD1;
        this->beepQueue[this->writeIndex].volume   = DISARMED_VOLUME1;
        incrementWriteIndex();

        /* Second beep. */
        this->beepQueue[this->writeIndex].duration = DISARMED_DURATION2;
        this->beepQueue[this->writeIndex].period   = DISARMED_PERIOD2;
        this->beepQueue[this->writeIndex].volume   = DISARMED_VOLUME2;
        incrementWriteIndex();
    }
    if (!isQueueFull()) {
        /* Delay. */
        this->beepQueue[this->writeIndex].duration = DISARMED_DELAY;
        this->beepQueue[this->writeIndex].period   = 0;
        this->beepQueue[this->writeIndex].volume   = 0;
        incrementWriteIndex();
    }
}

void BuzzerManager::addConfigurationBeeps(int numberOfBeeps) {
    for (int i = 0; i < numberOfBeeps; i++) {
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = CONFIG_DURATION;
            this->beepQueue[this->writeIndex].period   = CONFIG_PERIOD;
            this->beepQueue[this->writeIndex].volume   = CONFIG_VOLUME;
            incrementWriteIndex();
        }
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = CONFIG_DELAY;
            this->beepQueue[this->writeIndex].period   = 0;
            this->beepQueue[this->writeIndex].volume   = 0;
            incrementWriteIndex();
        }
    }
}

void BuzzerManager::addInitiatedBeeps(int numberOfBeeps) {
    for (int i = 0; i < numberOfBeeps; i++) {
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = INITIATED_DURATION;
            this->beepQueue[this->writeIndex].period   = INITIATED_PERIOD;
            this->beepQueue[this->writeIndex].volume   = INITIATED_VOLUME;
            incrementWriteIndex();
        }
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = INITIATED_DELAY;
            this->beepQueue[this->writeIndex].period   = 0;
            this->beepQueue[this->writeIndex].volume   = 0;
            incrementWriteIndex();
        }
    }
}

void BuzzerManager::addNavigationErrorBeeps(int numberOfBeeps) {
    for (int i = 0; i < numberOfBeeps; i++) {
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = NAVERROR_DURATION;
            this->beepQueue[this->writeIndex].period   = NAVERROR_PERIOD;
            this->beepQueue[this->writeIndex].volume   = NAVERROR_VOLUME;
            incrementWriteIndex();
        }
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = NAVERROR_DELAY;
            this->beepQueue[this->writeIndex].period   = 0;
            this->beepQueue[this->writeIndex].volume   = 0;
            incrementWriteIndex();
        }
    }
}

void BuzzerManager::addWarningBeeps(int numberOfBeeps) {
    for (int i = 0; i < numberOfBeeps; i++) {
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = WARNING_DURATION;
            this->beepQueue[this->writeIndex].period   = WARNING_PERIOD;
            this->beepQueue[this->writeIndex].volume   = WARNING_VOLUME;
            incrementWriteIndex();
        }
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = WARNING_DELAY;
            this->beepQueue[this->writeIndex].period   = 0;
            this->beepQueue[this->writeIndex].volume   = 0;
            incrementWriteIndex();
        }
    }
}

void BuzzerManager::clearBeepQueue() {
    for (int i = 0; i < QUEUE_SIZE; i++)
        this->beepQueue[i] = {0.0, 0, 0};

    this->readIndex           = 0;
    this->writeIndex          = 0;
    this->numInstructionsLeft = 0;
}

bool BuzzerManager::incrementReadIndex() {

    /* We cannot read another instruction if the queue is empty. */
    if (isQueueEmpty())
        return false;

    /* Increment read index. */
    this->readIndex++;
    this->numInstructionsLeft--;
    if (this->readIndex >= QUEUE_SIZE)
        this->readIndex = 0;
    return true;
}

bool BuzzerManager::incrementWriteIndex() {

    /* We cannot write another instruction if the queue is full. */
    if (isQueueFull())
        return false;

    /* Increment write index. */
    this->writeIndex++;
    this->numInstructionsLeft++;
    if (this->writeIndex >= QUEUE_SIZE)
        this->writeIndex = 0;
    return true;
}

bool BuzzerManager::isQueueEmpty() {
    return this->writeIndex == this->readIndex &&
           this->numInstructionsLeft == 0;
}

bool BuzzerManager::isQueueFull() {
    return this->writeIndex == this->readIndex &&
           this->numInstructionsLeft > 0;
}

void BuzzerManager::updateBuzzer(float currentTime) {

    /* If there's no instruction currently running... */
    float elapsedTime = currentTime - this->beepStartTime;
    if (!(isInstructionBusy &&
          elapsedTime <= this->currentInstruction.duration)) {

        /* ... and there are instructions left, then start the next one. */
        if (!isQueueEmpty()) {
            this->currentInstruction = this->beepQueue[readIndex];
            incrementReadIndex();
        }

        /* ... and there are no instructions left, turn off any sound. */
        else {
            this->currentInstruction = {0.0, 0, 0};
            isInstructionBusy        = false;
        }
    }

    /* Output the current instruction. */
    outputBuzzerPWM(currentInstruction.period, currentInstruction.volume);
}
