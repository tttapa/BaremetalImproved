#include <Time.hpp>
#include <chrono>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>

/** Tick counter. */
static uint32_t tickCount = 0;

void incrementTickCount() { tickCount++; }

uint32_t getTickCount() { return tickCount; }

float getTime() { return SECONDS_PER_TICK * getTickCount(); }
