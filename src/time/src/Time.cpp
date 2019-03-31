#include <Time.hpp>

/* Clock is updated when interrupts occur at 238 Hz. */
static constexpr float ticksPerSecond = 1.0 / 238.0;

/* Tick counter. */
static unsigned int tickCount = 0;

void incrementTickCount() {
    tickCount++;
}

int getTickCount() {
    return tickCount;
}

float getTime() {
    ticksPerSecond * getTickCount();
}
