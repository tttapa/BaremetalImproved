#include <Globals.hpp>
#include <GradualThrustChangeManager.hpp>
#include <MiscInstances.hpp>

const real_t GTC_DURATION =
    (TICKS_PER_SECOND * 1);  //*** 1000ms from u_hover --> u_thrust_joystick

void GradualThrustChangeManager::init() {
    this->busy  = false;
    this->counter = 0;
    this->thrust  = 0; 
}

void GradualThrustChangeManager::start(real_t startThrust) {
    this->busy = true;
    this->counter = 0;
    this->thrust = startThrust;
}

void GradualThrustChangeManager::update() {

    // Compute the number of ticks left.
    real_t ticksLeft = (real_t)(GTC_DURATION - this->counter);

    // There are ticks left.
    if (this->counter < GTC_DURATION) {
        // Lineair interpolation.
        this->thrust += (rcManager.getThrottle() - this->thrust) / ticksLeft;

    // No ticks left. --> Manager is ready.
    } else {
        this->busy = 0;
    }

    // Increment the counter.
    this->counter += 1;
}