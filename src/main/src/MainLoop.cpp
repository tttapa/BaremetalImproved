#include <platform/Interrupt.hpp>
#include <MainInterrupt.hpp>

void mainLoop() {
    if (doInterrupt) {
        updateFSM();
        doInterrupt = false;
    }
}