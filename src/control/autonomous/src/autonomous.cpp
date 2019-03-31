// Original: BareMetal/src/control/timers.c (updates auto-fsm + calls appropriate state action)
#include <autonomous.hpp>


void initializeAutonomousFSM() {

    /* Start in IDLE. */
    
}

AutonomousOutput updateAutonomousFSM() {

    // TODO: update FSM
    // TODO: use accelerometer in takeoff stage 1 & landing stage 2!!!
    /**
     * 
     * --- WIRELESS POWER TRANSFER ---
     * Idle -> WPT
     * WPT -> Idle
     * 
     * ---------- MAIN CYCLE ----------
     * Idle -> Takeoff
     * Takeoff -> Loitering
     * Loitering -> Navigating
     * Navigating -> Landing
     * Landing -> Idle
     * 
     * ------------ ERROR ------------
     * Takeoff Stage 2 -> Error
     * Loitering -> Error
     * Navigating -> Error
     * Landing Stage 1 -> Error
     * 
     * 
     */


}

