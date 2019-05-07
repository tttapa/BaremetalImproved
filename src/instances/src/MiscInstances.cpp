#include <MiscInstances.hpp>

/** Instance of the armed manager. */
ArmedManager armedManager;

/** Instance of the buzzer manager. */
BuzzerManager buzzerManager;

/** Instance of the configuration manager. */
ConfigurationManager configManager;

/** Instance of the ESC statup script. */
ESCStartupScript escStartupScript;

/** Instance of the gradual thrust change manager. */
GradualThrustChangeManager gtcManager;

/** Instance of the RC manager. */
RCManager rcManager;

void initMiscInstances() {
    armedManager.init();
    buzzerManager.init();
    configManager.init();
    escStartupScript.init(true);
    gtcManager.init();
    rcManager.init();
}