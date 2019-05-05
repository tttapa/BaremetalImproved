#include "Time.hpp"

/** Clock is updated when interrupts occur at 238 Hz. */
static constexpr real_t secondsPerTick = 1.0 / 238.0;

/** Tick counter. */
static unsigned int tickCount = 0;

void incrementTickCount() {
    tickCount++;
}

real_t getSecondsPerTick() {
    return secondsPerTick;
}

int getTickCount() {
    return tickCount;
}

real_t getTime() {
    return secondsPerTick * getTickCount();
}
