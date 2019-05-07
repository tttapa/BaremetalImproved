#include <ESCStartupScript.hpp>
#include <Time.hpp>

/** The startup script lasts 5.0 seconds. */
const float STARTUP_DURATION = 5.0;

/** The shutdown script lasts 0.5 seconds. */
const float SHUTDOWN_DURATION = 0.50;

/**
 * The startup script begins as soon as the common thrust exceeds 0.105, or a
 * 50% PWM duty cycle. The shutdown script will begin as soon as it goes below
 * this value. During the shutdown script this value will be held for 0.5 s.
 */
const float COMMON_THRUST_THRESHOLD = 0.105;

/**
 * During the duration of the startup script, a common thrust of 0.215 or a 55%
 * PWM duty cycle will be sent to the ESCs.
 */
const float STARTUP_COMMON_THRUST = 0.215;

real_t ESCStartupScript::update(real_t commonThrust) {

    /* If the script is disabled, bypass this script. */
    if (!this->enabled)
        return commonThrust;

    /* Begin the startup script if the ESCs are not running, and the pilot
       raises the throttle enough. */
    if (!escsRunning && commonThrust >= COMMON_THRUST_THRESHOLD) {
        escsRunning   = true;
        startupActive  = true;
        startupStartTime = getTime();
    }

    /* Begin the shutdown script if the ESCs are running, and the pilot lowers
       the throttle enough. */
    if (escsRunning && commonThrust < COMMON_THRUST_THRESHOLD) {
        escsRunning    = true;
        shutdownActive  = true;
        shutdownStartTime = getTime();
    }

    /* Start up script active? */
    if (startupActive) {
        /* Continue script. */
        if (getTime() - startupStartTime <= STARTUP_DURATION)
            return STARTUP_COMMON_THRUST;

        /* End script. */
        startupActive = false;
        escsRunning  = true;
    }

    /* Shutdown script active? */
    if (shutdownActive) {
        /* Continue script. */
        if (getTime() - shutdownStartTime <= SHUTDOWN_DURATION)
            return COMMON_THRUST_THRESHOLD;

        /* End script. */
        shutdownActive = false;
        escsRunning   = false;
    }

    /* No scripts are active, so are the ESCs running? */
    if (escsRunning)
        return commonThrust; /* Send given signal to "common motor". */
    else
        return 0.0; /* Turn off the "common motor". */
}