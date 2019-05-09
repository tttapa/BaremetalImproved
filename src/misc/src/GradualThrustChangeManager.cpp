#include <GradualThrustChangeManager.hpp>

/* Includes from src. */
#include <MiscInstances.hpp>
#include <RCValues.hpp>
#include <Time.hpp>

/** Gradual thrust change lasts 1.0 seconds. */
const real_t GTC_DURATION = TICKS_PER_SECOND * 1.0;

void GradualThrustChangeManager::init() {
    this->busy    = false;
    this->counter = 0;
    this->thrust  = 0;
}

void GradualThrustChangeManager::start(real_t startThrust) {
    this->busy    = true;
    this->counter = 0;
    this->thrust  = startThrust;
}

void GradualThrustChangeManager::update() {

    /* Compute the number of ticks left. */
    real_t ticksLeft = (real_t)(GTC_DURATION - this->counter);

    /* If there are ticks left... */
    if (this->counter < GTC_DURATION)
        /* Linear interpolation. */
        this->thrust += (getThrottle() - this->thrust) / ticksLeft;
    else
        /* Gradual thrust change is finished. */
        this->busy = 0;

    /* Increment the counter. */
    this->counter += 1;
}