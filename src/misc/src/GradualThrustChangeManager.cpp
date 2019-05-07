#include <Globals.hpp>
#include <GradualThrustChangeManager.hpp>

bool GradualThrustChangeManager::isBusy() { return this->isBusy; }

real_t GradualThrustChangeManager::getThrust() { return this->thrust; }

void GradualThrustChangeManager::init() {
    this->isBusy  = true;
    this->counter = 0;
    this->thrust  = alt_thrust; //TODO: hoe dit invullen? 
}

void GradualThrustChangeManager::update() {
    real_t ticksLeft = (real_t)(GTC_DURATION - this->counter);
    if (this->counter < GTC_DURATION) {
        //TODO: deze thrust komt van de RC.
        this->thrust += (getRCThrottle() - this->thrust) / ticksLeft;
    } else {
        this->isBusy = 0;
    }
    this->counter += 1;
}