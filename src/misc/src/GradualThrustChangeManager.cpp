#include <Globals.hpp>
#include <GradualThrustChangeManager.hpp>

bool GradualThrustChangeManager::isBusy() { return this->busy; }

real_t GradualThrustChangeManager::getThrust() { return this->thrust; }

void GradualThrustChangeManager::init() {
    this->busy  = true;
    this->counter = 0;
    this->thrust  = 0; //TODO: hoe dit invullen? 
}

void GradualThrustChangeManager::update() {
    real_t ticksLeft = (real_t)(GTC_DURATION - this->counter);
    if (this->counter < GTC_DURATION) {
        //TODO: deze thrust komt van de RC.
        this->thrust += (getRCThrottle() - this->thrust) / ticksLeft;
    } else {
        this->busy = 0;
    }
    this->counter += 1;
}