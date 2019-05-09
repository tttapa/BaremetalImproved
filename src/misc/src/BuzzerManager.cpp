#include <BuzzerManager.hpp>

/* Includes from src. */
#include <Time.hpp>

/* Includes from src-vivado. */
#include <output/Buzzer.hpp>

/** Armed beep 1 lasts 0.30 seconds. */
static constexpr float ARMED_DURATION1 = 0.30;
/** Armed beep 1 has a low pitch. */
static constexpr int ARMED_PERIOD1 = 0x37800;
/** Armed beep 1 has a medium volume. */
static constexpr int ARMED_VOLUME1 = 0x20000;
/** Armed beep 2 lasts 0.30 seconds. */
static constexpr float ARMED_DURATION2 = 0.30;
/** Armed beep 2 has a medium pitch. */
static constexpr int ARMED_PERIOD2 = 0x27800;
/** Armed beep 2 has a medium volume. */
static constexpr int ARMED_VOLUME2 = 0x20000;
/** Armed beep is followed by 0.40 seconds of silence. */
static constexpr float ARMED_DELAY = 0.40;

/** Configuration beep lasts 0.25 seconds. */
static constexpr float CONFIG_DURATION = 0.25;
/** Configuration beep has a low pitch. */
static constexpr int CONFIG_PERIOD = 0x35000;
/** Configuration beep is as loud as possible. */
static constexpr int CONFIG_VOLUME = 0x32000;
/** Configuration beeps have 0.20 seconds between them. */
static constexpr float CONFIG_DELAY = 0.20;

/** Disarmed beep 1 lasts 0.30 seconds. */
static constexpr float DISARMED_DURATION1 = 0.30;
/** Disarmed beep 1 has a medium pitch. */
static constexpr int DISARMED_PERIOD1 = 0x27800;
/** Disarmed beep 1 has a medium volume. */
static constexpr int DISARMED_VOLUME1 = 0x20000;
/** Disarmed beep 2 lasts 0.30 seconds. */
static constexpr float DISARMED_DURATION2 = 0.30;
/** Disarmed beep 2 has a low pitch. */
static constexpr int DISARMED_PERIOD2 = 0x37800;
/** Disarmed beep 2 has a medium volume. */
static constexpr int DISARMED_VOLUME2 = 0x20000;
/** Disarmed beep is followed by 0.40 seconds of silence. */
static constexpr float DISARMED_DELAY = 0.40;

/** Drone initiated has 3 beeps. */
static constexpr int NUM_INITIATED_BEEPS = 3;
/** Initiated beep lasts 0.05 seconds. */
static constexpr float INITIATED_DURATION = 0.05;
/** Initiated beep has a medium pitch. */
static constexpr int INITIATED_PERIOD = 0x27800;
/** Initiated beep has a loud volume. */
static constexpr int INITIATED_VOLUME = 0x25000;
/** Initiated beeps have 0.01 seconds between them. */
static constexpr int INITIATED_DELAY = 0.01;

/** Warning for changing configuration has 2 beeps. */
static constexpr int NUM_WARNING_BEEPS = 2;
/** Warning beep lasts 0.12 seconds. */
static constexpr float WARNING_DURATION = 0.12;
/** Warning beep has a high pitch. */
static constexpr int WARNING_PERIOD = 0x21000;
/** Warning beep has a medium volume. */
static constexpr int WARNING_VOLUME = 0x20000;
/** Warning beeps have 0.06 seconds between them. */
static constexpr float WARNING_DELAY = 0.06;

/** Navigation error beep lasts 0.50 seconds. */
static constexpr float NAVERROR_DURATION = 0.5;
/** Navigation error beep has a medium pitch. */
static constexpr int NAVERROR_PERIOD = 0x25000;
/** Navigation error beep is as loud as possible. */
static constexpr int NAVERROR_VOLUME = 0x30000;
/** Navigation error beeps have 0.50 seconds between them. */
static constexpr float NAVERROR_DELAY = 0.5;

void BuzzerManager::addArmedBeeps() {
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

void BuzzerManager::addDisarmedBeeps() {
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

void BuzzerManager::addInitiatedBeeps() {
    for (int i = 0; i < NUM_INITIATED_BEEPS; i++) {
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

void BuzzerManager::addWarningBeeps() {
    for (int i = 0; i < NUM_WARNING_BEEPS; i++) {
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
    return this->writeIndex == this->readIndex && this->numInstructionsLeft > 0;
}

bool BuzzerManager::tryAddingNavigationErrorBeep() {
    if (getNumInstructionsUntilFull() <= 2)
        return false;

    /* Add the navigation error beep. */
    this->beepQueue[this->writeIndex].duration = NAVERROR_DURATION;
    this->beepQueue[this->writeIndex].period   = NAVERROR_PERIOD;
    this->beepQueue[this->writeIndex].volume   = NAVERROR_VOLUME;
    incrementWriteIndex();

    /* Add the delay after the beep. */
    this->beepQueue[this->writeIndex].duration = NAVERROR_DELAY;
    this->beepQueue[this->writeIndex].period   = 0;
    this->beepQueue[this->writeIndex].volume   = 0;
    incrementWriteIndex();

    /* Successfully added navigation error beep. */
    return true;
}

void BuzzerManager::update() {

    /* If there's no instruction currently running... */
    float elapsedTime = getTime() - this->beepStartTime;
    if (!(this->instructionBusy &&
          elapsedTime <= this->currentInstruction.duration)) {

        /* ... and there are instructions left, then start the next one. */
        if (!isQueueEmpty()) {
            this->currentInstruction = this->beepQueue[readIndex];
            incrementReadIndex();
        }

        /* ... and there are no instructions left, turn off any sound. */
        else {
            this->currentInstruction = {0.0, 0, 0};
            this->instructionBusy    = false;
        }
    }

    /* Output the current instruction. */
    outputBuzzerPWM(currentInstruction);
}
