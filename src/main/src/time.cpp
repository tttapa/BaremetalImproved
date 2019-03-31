#include <time.hpp>

/* Clock is updated when interrupts occur at 238 Hz. */
static const float ticksPerSecond = 1.0 / 238.0;

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
