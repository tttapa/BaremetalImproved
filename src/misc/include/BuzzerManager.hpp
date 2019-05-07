// Original: BareMetal/src/control/timers.h
#include <Time.hpp>

/**
 * The beep queue has a maximum size of 30 instructions. If the buffer is full
 * and the user tries to add another instruction, the buffer will not change.
 */
const int QUEUE_SIZE = 30;

/** Armed beep 1 lasts 0.30 seconds. */
const float ARMED_DURATION1 = 0.30;
/** Armed beep 1 has a low pitch. */
const int ARMED_PERIOD1 = 0x37800;
/** Armed beep 1 has a medium volume. */
const int ARMED_VOLUME1 = 0x20000;
/** Armed beep 2 lasts 0.30 seconds. */
const float ARMED_DURATION2 = 0.30;
/** Armed beep 2 has a medium pitch. */
const int ARMED_PERIOD2 = 0x27800;
/** Armed beep 2 has a medium volume. */
const int ARMED_VOLUME2 = 0x20000;
/** Armed beep is followed by 0.40 seconds of silence. */
const float ARMED_DELAY = 0.40;

/** Configuration beep lasts 0.25 seconds. */
const float CONFIG_DURATION = 0.25;
/** Configuration beep has a low pitch. */
const int CONFIG_PERIOD = 0x35000;
/** Configuration beep is as loud as possible. */
const int CONFIG_VOLUME = 0x32000;
/** Configuration beeps have 0.20 seconds between them. */
const float CONFIG_DELAY = 0.20;

/** Disarmed beep 1 lasts 0.30 seconds. */
const float DISARMED_DURATION1 = 0.30;
/** Disarmed beep 1 has a medium pitch. */
const int DISARMED_PERIOD1 = 0x27800;
/** Disarmed beep 1 has a medium volume. */
const int DISARMED_VOLUME1 = 0x20000;
/** Disarmed beep 2 lasts 0.30 seconds. */
const float DISARMED_DURATION2 = 0.30;
/** Disarmed beep 2 has a low pitch. */
const int DISARMED_PERIOD2 = 0x37800;
/** Disarmed beep 2 has a medium volume. */
const int DISARMED_VOLUME2 = 0x20000;
/** Disarmed beep is followed by 0.40 seconds of silence. */
const float DISARMED_DELAY = 0.40;

/** Initiated beep lasts 0.05 seconds. */
const float INITIATED_DURATION = 0.05;
/** Initiated beep has a medium pitch. */
const int INITIATED_PERIOD = 0x27800;
/** Initiated beep has a loud volume. */
const int INITIATED_VOLUME = 0x25000;
/** Initiated beeps have 0.01 seconds between them. */
const int INITIATED_DELAY = 0.01;

/** Warning beep lasts 0.12 seconds. */
const float WARNING_DURATION = 0.12;
/** Warning beep has a high pitch. */
const int WARNING_PERIOD = 0x21000;
/** Warning beep has a medium volume. */
const int WARNING_VOLUME = 0x20000;
/** Warning beeps have 0.06 seconds between them. */
const float WARNING_DELAY = 0.06;

/** Navigation error beep lasts 0.50 seconds. */
const float NAVERROR_DURATION = 0.5;
/** Navigation error beep has a medium pitch. */
const int NAVERROR_PERIOD = 0x25000;
/** Navigation error beep is as loud as possible. */
const int NAVERROR_VOLUME = 0x30000;
/** Navigation error beeps have 0.50 seconds between them. */
const float NAVERROR_DELAY = 0.5;

/**
 * Instruction to be sent to the buzzer containing a duration (float), a buzzer
 * period (int) and a buzzer volume (int).
 */
struct BuzzerInstruction {
    float duration;  ///< Duration of instruction in seconds.
    int period;      ///< Period of sound, represented as an integer.
    int volume;      ///< Volume of sound, represented as an integer.
};

class BuzzerManager {

  private:
    /**
     * Beep queue of {duration, period, volume}. For delays, period and volume
     * should be zero.
     */
    BuzzerInstruction beepQueue[QUEUE_SIZE];

    /** Current instruction being played on the buzzer. */
    BuzzerInstruction currentInstruction;

    /** Whether there is an instruction playing on the buzzer. */
    bool isInstructionBusy;

    /** Index in the beep queue where the next instruction will be placed. */
    int writeIndex;

    /** Index in the beep queue where the next instruction will be read. */
    int readIndex;

    /** Number of unread instructions in the beep queue. */
    int numInstructionsLeft;

    /** Time at which the current beep started playing. */
    float beepStartTime;

  public:
    /**
     * Adds a pair of "armed" beeps to the beep queue.
     */
    void addArmedBeep();

    /**
     * Adds a pair of "disarmed" beeps to the beep queue.
     */
    void addDisarmedBeep();

    /**
     * Adds the given number of configuration beeps to the beep queue.
     * 
     * @param   numberOfBeeps
     *          Number of configuration beeps to add to the beep queue.
     */
    void addConfigurationBeeps(int numberOfBeeps);

    /**
     * Adds the given number of initiated beeps to the beep queue.
     * 
     * @param   numberOfBeeps
     *          Number of initiated beeps to add to the beep queue.
     */
    void addInitiatedBeeps(int numberOfBeeps);

    /**
     * Adds the given number of navigation error beeps to the beep queue.
     * 
     * @param   numberOfBeeps
     *          Number of navigation error beeps to add to the beep queue.
     */
    void addNavigationErrorBeeps(int numberOfBeeps);

    /**
     * Adds the given number of warning beeps to the beep queue.
     * 
     * @param   numberOfBeeps
     *          Number of warning beeps to add to the beep queue.
     */
    void addWarningBeeps(int numberOfBeeps);

    /**
     * Clears the beep queue.
     */
    void clearBeepQueue();

    /**
     * Returns the number of instructions left in the beep queue.
     */
    int getNumInstructionsLeft() { return this->numInstructionsLeft; }

    /**
     * Returns the number of instructions that can still be added to the beep
     * queue before it is full.
     */
    int getNumInstructionsUntilFull() {
        return QUEUE_SIZE - this->numInstructionsLeft;
    }

    /**
     * Try incrementing the read index, wrapping if we've reached the end of the
     * beep queue.
     * 
     * @return  true
     *          If the buffer was not empty, so the read index was successfully
     *          incremented.
     * @return  false
     *          If the buffer was empty, so the read index was not incremented.
     */
    bool incrementReadIndex();

    /**
     * Try incrementing the write index, wrapping if we've reached the end of
     * the beep queue.
     * 
     * @return  true
     *          If the buffer was not full, so the write index was successfully
     *          incremented.
     * @return  false
     *          If the buffer was full, so the write index was not incremented.
     */
    bool incrementWriteIndex();

    /**
     * Check whether the beep queue is empty.
     * 
     * @return  true
     *          If the beep queue is empty.
     * @return  false
     *          Otherwise.
     */
    bool isQueueEmpty();

    /**
     * Check whether the beep queue is full.
     * 
     * @return  true
     *          If the beep queue is full.
     * @return  false
     *          Otherwise.
     */
    bool isQueueFull();

    /**
     * Updates the signal sent to the buzzer and the beep queue based on the
     * current time.
     * 
     * @param   currentTime
     *          Current time in seconds.
     */
    void updateBuzzer(float currentTime);
};
