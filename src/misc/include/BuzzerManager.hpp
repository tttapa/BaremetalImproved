// Original: BareMetal/src/control/timers.h
#include <Time.hpp>

/**
 * The beep queue has a maximum size of 30 instructions. If the buffer is full
 * and the user tries to add another instruction, the buffer will not change.
 */
const int QUEUE_SIZE = 30;

/**
 * Instruction to be sent to the buzzer containing a duration (float), a buzzer
 * period (int) and a buzzer volume (int).
 */
struct BuzzerInstruction {
    float duration;  ///< Duration of instruction in seconds.
    int period;      ///< Period of sound, represented as an integer.
    int volume;      ///< Volume of sound, represented as an integer.
};

/**
 * Class to manage the sounds produced by the buzzer. To do this, it has a
 * queue of BuzzerInstructions that it will play while there are still
 * instructions left to play.
 */
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
    bool instructionBusy;

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
    void addArmedBeeps();

    /**
     * Adds a pair of "disarmed" beeps to the beep queue.
     */
    void addDisarmedBeeps();

    /**
     * Adds the given number of configuration beeps to the beep queue.
     * 
     * @param   numberOfBeeps
     *          Number of configuration beeps to add to the beep queue.
     */
    void addConfigurationBeeps(int numberOfBeeps);

    /**
     * Adds 3 initiated beeps to the beep queue.
     */
    void addInitiatedBeeps();

    /**
     * Adds 2 warning beeps to the beep queue.
     */
    void addWarningBeeps();

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
     * Return whether there is currently an instruction playing on the buzzer.
     */
    bool isInstructionBusy() { return this->instructionBusy; }

    /**
     * Check whether the beep queue is empty.
     * 
     * @return  true
     *          If the beep queue is empty.
     * @return  false
     *          Otherwise.s
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
     * Tries adding a navigation error beep to the queue. This is only possible
     * if both the beep instruction and the delay instruction fit in the queue.
     * 
     * @return  true
     *          If the navigation beep was successfully added.
     * @return  false
     *          If the beep queue could not have fit both the navigation beep
     *          and the following delay, so nothing was added.
     */
    bool tryAddingNavigationErrorBeep();

    /**
     * Updates the signal sent to the buzzer and the beep queue based on the
     * current time.
     * 
     * @param   currentTime
     *          Current time in seconds.
     */
    void updateBuzzer(float currentTime);
};
