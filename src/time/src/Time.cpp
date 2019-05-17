#include <Time.hpp>
#include <chrono>

/* Includes from src-vivado. */
#include <PublicHardwareConstants.hpp>

/** Tick counter. */
static uint32_t tickCount = 0;

void incrementTickCount() { tickCount++; }

uint32_t getTickCount() { return tickCount; }

float getTime() { return SECONDS_PER_TICK * getTickCount(); }

uint64_t getMillis() {
    auto now = chrono::system_clock::now().time_since_epoch();
    return chrono::duration_cast<chrono::milliseconds>(now).count();
}
