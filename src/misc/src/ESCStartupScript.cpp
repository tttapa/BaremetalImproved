#include <ESCStartupScript.hpp>

/* Includes from src. */
#include <MiscInstances.hpp>
#include <Time.hpp>

void ESCStartupScript::init(bool enabled) {
    this->enabled        = enabled;
    this->escsRunning    = false;
    this->shutdownActive = false;
    this->startupActive  = false;
}

real_t ESCStartupScript::update(real_t commonThrust) {

    /* If the script is disabled, bypass this script. */
    if (!this->enabled)
        return commonThrust;

    /* Begin the startup script if the ESCs are not running, and the pilot
       raises the throttle enough. */
    if (!escsRunning && commonThrust >= COMMON_THRUST_THRESHOLD) {
        escsRunning      = true;
        startupActive    = true;
        startupStartTime = getTime();
    }

    /* Begin the shutdown script if the ESCs are running, and the pilot lowers
       the throttle enough. */
    if (escsRunning && commonThrust < COMMON_THRUST_THRESHOLD) {
        escsRunning       = true;
        shutdownActive    = true;
        shutdownStartTime = getTime();
    }

    /* Start up script active? */
    if (startupActive) {
        /* Continue script. */
        if (getTime() - startupStartTime <= STARTUP_DURATION)
            return STARTUP_COMMON_THRUST;

        /* End script. */
        startupActive = false;
        escsRunning   = true;

        /* Start gradual thrust change from the current common thrust. */
        // gtcManager.start(STARTUP_COMMON_THRUST);
    }

    /* Shutdown script active? */
    if (shutdownActive) {
        /* Continue script. */
        if (getTime() - shutdownStartTime <= SHUTDOWN_DURATION)
            return COMMON_THRUST_THRESHOLD;

        /* End script. */
        shutdownActive = false;
        escsRunning    = false;
    }

    /* No scripts are active, so are the ESCs running? */
    if (escsRunning)
        return commonThrust; /* Send given signal to "common motor". */
    else
        return 0.0; /* Turn off the "common motor". */
}