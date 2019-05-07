#pragma once
#include <real_t.h>

/**
 * A class to manage the gradual thrust change. Gradual thrust change gives the
 * pilot an extra second to react when switching from "altitude-hold mode" to
 * "manual mode". Because the throttle is used to adjust the height during
 * "altitude-hold mode", the pilot might not remember exactly where the hovering
 * thrust was. Therefore, when switching back to "manual mode", the thrust will
 * gradually evolve from the hovering thrust to the thrust given by the throttle
 * stick.
 */
class GradualThrustChangeManager {

  private:

    /** A boolean representing whether the gradual thrust change is busy. */
    bool busy;

    /**
     * A counter to keep track of how long the gradual thrust change has been
     * busy.
     */
    int counter;

    /** The common thrust as a result of the gradual thrust change. */
    real_t thrust;

  public:
    /**
     * Return whether the gradual thrust change manager is busy.
     */
    bool isBusy() { return this->busy; };

    /**
     * Return the common thrust as a result of the gradual thrust change.
     */
    real_t getThrust() { return this->thrust; };

    /**
     * Start the gradual thrust change. The gradual thrust change manager's
     * common thrust will (linearly) evolve from the given start thrust to the
     * thrust defined by the RC.
     * 
     * @param   startThrust
     *          The initial thrust to assign to the thrust of the gradual thrust 
     *          change manager when it's started.
     */
    void start(real_t startThrust);

    /** 
     * Initialize the gradual thrust change manager.
     */
    void init();

    /**
     * Update the gradual thrust change manager. If there's still time left for 
     * for the gradual thrust change manager to execute, it continues its action
     * and adapts it's thrust. If the time has expired, it's busy boolean is 
     * set to 'false'.
     */
    void update();
};
