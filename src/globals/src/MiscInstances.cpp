#include <MiscInstances.hpp>

/** Instance of the buzzer manager. */
BuzzerManager buzzerManager;

/** Instance of the configuration manager. */
ConfigurationManager configManager;

/** Instance of the ESC statup script. */
ESCStartupScript escStartupScript;

/** Instance of the gradual thrust change manager. */
GradualThrustChangeManager gtcManager;

void initMiscInstances() {
    buzzerManager.init();
    configManager.init();
    escStartupScript.init(true);
    gtcManager.init();
}