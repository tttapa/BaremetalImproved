#include <real_t.h>

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

/**
 * Class to manage the startup and shutdown of the ESCs. When commercial ESCs
 * are used, this class should be disabled. In this case the update() method
 * will simply return the given common thrust, so it can still be used. When the
 * class is enabled, it will manage the startup and shutdown script and return
 * the appropriate signal to send to the common motor.
 */
class ESCStartupScript {
  private:
    /** Whether the ESCs are currently running. */
    bool escsRunning;

    /** Whether the ESC startup script is enabled. */
    bool enabled;

    /** Whether the shutdown script is active. */
    bool shutdownActive;

    /** Whether the startup script is active. */
    bool startupActive;

    /**
     * Time at which the ESC startup began. This will allow the startup script
     * to keep track of how long it's been busy.
     */
    real_t startupStartTime;

    /**
     * Time at which the ESC shutdown began. This adds a small delay to the
     * shutdown of the ESCs in order to prevent an accidental shutdown mid
     * flight.
     */
    real_t shutdownStartTime;

  public:
    /**
     * Returns whether the ESCs are currently running.
     */
    bool areESCsRunning() { return this->escsRunning; }

    /**
     * Disable the ESC startup script, and send the attitude control signal
     * immediately to the ESCs instead.
     */
    void disable() { this->enabled = false; }

    /**
     * Enable the ESC startup script, so the ESCs will start up safely
     * before the actual motor signals are sent.
     */
    void enable() { this->enabled = true; }

    /**
     * Reset the ESC startup script with the given enabled status.
     * 
     * @param   enabled
     *          Whether the startup script should be enabled.
     */
    void init(bool enabled);

    /**
     * Returns whether the startup script is enabled.
     */
    bool isEnabled() { return this->enabled; }

    /**
     * Returns whether the shutdown script is active.
     */
    bool isShutdownActive() { return this->shutdownActive; }

    /**
     * Returns whether the startup script is active.
     */
    bool isStartupActive() { return this->startupActive; }

    /**
     * Update the ESC startup script if enabled.
     * 
     * @param   commonThrust
     *          The signal that is would currently be sent to the "common
     *          motor".
     * 
     * @return  The signal to be sent to the "common motor" according to the ESC
     *          startup script, if the script is enabled.
     * @return  The given signal, if the script is disabled.
     */
    real_t update(real_t commonThrust);
};