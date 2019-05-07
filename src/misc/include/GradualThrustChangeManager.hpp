#pragma once
#include <Time.hpp>

/**
 * A class to manage the gradual thrust change. Gradual
 * thrust change gives the pilot an extra second to react when switching from
 * "altitude-hold mode" to "manual mode". Because the throttle is used to adjust
 * the height during "altitude-hold mode", the pilot might not remember exactly
 * where the hovering thrust was. Therefore, when switching back to 
 * "manual mode", the thrust will gradually evolve from the hovering thrust 
 * to the thrust given by the throttle stick.
 */
class GradualThrustChangeManager {

  private:

    /**
     * A boolean representing whether the gradual thrust change manager is 
     * still busy.
     */
    bool busy;

    /**
     * A counter that counts the number of times the gradual thrust change 
     * manager has been updated.
     */
    int counter;

    /** The thrust as a result from gradual thrust change. */
    real_t thrust;

  public:
    /**
     * A function that returns whether the gradual thrust change manager is 
     * still busy.
     */
    bool isBusy() { return this->busy; };

    /**
     * A function that returns the thrust as a result from the gradual thrust 
     * change.
     */
    real_t getThrust() { return this->thrust; };

    /**
     * A function that starts the gradual thrust change manager. The manager is 
     * set to 'busy', it's counter is set to zero and it's thrust is set to the 
     * given startThrust.
     * 
     * @param   startThrust
     *          The initial thrust to assign to the thrust of the gradual thrust 
     *          change manager, when it's started.
     */
    void start(real_t startThrust);

    /** 
     * Initialize the gradual thrust change manager. The manager is not yet 
     * busy and it's counter and thrust are set to zero.
     */
    void init();

    /**
     * Update the gradual thrust change manager. If there's still time left for 
     * for the gradual thrust change manager to execute, it continues its action
     * and adapts it's thrust. If the time has expired, it's busy boolean is 
     * set to 'false'. The counter is incremented.
     */
    void update();
};
