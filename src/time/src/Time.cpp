#include <Time.hpp>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>

/** Tick counter. */
static uint32_t tickCount = 0;

void incrementTickCount() { tickCount++; }

uint32_t getTickCount() { return tickCount; }

float getTime() { return SECONDS_PER_TICK * getTickCount(); }

uint64_t getMillis() { return (uint64_t)(getTime() * 1000.0); }
