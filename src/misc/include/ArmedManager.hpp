#pragma once

/* Includes from src. */
#include <real_t.h>

/**
 * Class to control whether the drone is armed or disarmed. To arm the drone,
 * the pilot must hold the left joystick in the bottom-right corner for 2
 * seconds when in the manual flight mode. Arming an already armed drone is no
 * big deal. To disarm the drone, the pilot must hold the left joystick in the
 * bottom-left corner for 2 seconds when in manual flight mode. Disarming an
 * already disarmed drone is also no big deal.
 * 
 * If the drone is disarmed, no signals will be sent to the motors.
 */
class ArmedManager {
  private:
    /** Whether the drone is currently armed. */
    bool armed = false;

    /**
     * Whether the left joystick is in one of the "changing arm status" zones,
     * waiting to either arm or disarm the drone.
     */
    bool isWaitingForChange = false;

    /** Time at which the drone started waiting to arm or disarm. */
    real_t waitingStartTime = 0.0;

  public:
    /**
     * Initialize the armed manager and set the armed status to disarmed.
     */
    void init() { this->armed = false; }

    /**
     * Returns whether the drone is currently armed.
     */
    bool isArmed() { return this->armed; }

    /**
     * Update the armed manager. If the pilot holds the left joystick in the
     * bottom-right corner for 2 seconds, the drone will be armed. If the pilot
     * holds it in the bottom-left corner for 2 seconds, the drone will be
     * disarmed.
     */
    void update();
};