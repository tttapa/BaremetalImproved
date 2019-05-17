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
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}
