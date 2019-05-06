#include "Time.hpp"

/** Tick counter. */
static unsigned int tickCount = 0;

void incrementTickCount() { tickCount++; }

int getTickCount() { return tickCount; }

real_t getTime() { return SECONDS_PER_TICK * getTickCount(); }
