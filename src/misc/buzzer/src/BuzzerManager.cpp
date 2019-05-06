// Original: BareMetal/src/control/buzzercodesiel.c
#include <Buzzer.hpp>
#include <BuzzerManager.hpp>

void BuzzerManager::addArmedBeep() {}

void BuzzerManager::addConfigurationBeeps(int numberOfBeeps) {
    for (int i = 0; i < numberOfBeeps; i++) {
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = BEEP_CONFIG_DURATION;
            this->beepQueue[this->writeIndex].period   = BEEP_CONFIG_PERIOD;
            this->beepQueue[this->writeIndex].volume   = BEEP_CONFIG_VOLUME;
            incrementWriteIndex();
        }
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = BEEP_CONFIG_DELAY;
            this->beepQueue[this->writeIndex].period   = 0;
            this->beepQueue[this->writeIndex].volume   = 0;
            incrementWriteIndex();
        }
    }
}

void BuzzerManager::addWarningBeeps(int numberOfBeeps) {
    for (int i = 0; i < numberOfBeeps; i++) {
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = BEEP_WARNING_DURATION;
            this->beepQueue[this->writeIndex].period   = BEEP_WARNING_PERIOD;
            this->beepQueue[this->writeIndex].volume   = BEEP_WARNING_VOLUME;
            incrementWriteIndex();
        }
        if (!isQueueFull()) {
            this->beepQueue[this->writeIndex].duration = BEEP_WARNING_DELAY;
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

bool BuzzerManager::isQueueEmpty() {
    return this->writeIndex == this->readIndex &&
           this->numberInstructionsLeft == 0
}

bool BuzzerManager::isQueueFull() {
    return this->writeIndex == this->readIndex &&
           this->numberInstructionsLeft > 0
}

bool BuzzerManager::incrementReadIndex() {

    /* We cannot read another instruction if the queue is empty. */
    if (isQueueEmpty())
        return false;

    /* Increment read index. */
    this->readIndex++;
    this->numberInstructionsLeft--;
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
    this->numberInstructionsLeft++;
    if (this->writeIndex >= QUEUE_SIZE)
        this->writeIndex = 0;
    return true;
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
