#include <MiscInstances.hpp>

/** Instance of the armed manager. */
ArmedManager armedManager;

/** Instance of the input bias manager. */
BiasManager biasManager;

/** Instance of the buzzer manager. */
BuzzerManager buzzerManager;

/** Instance of the configuration manager. */
ConfigurationManager configManager;

/** Instance of the ESC statup script. */
// ESCStartupScript escStartupScript;

/** Instance of the gradual thrust change manager. */
//GradualThrustChangeManager gtcManager;

void initMiscInstances() {
    armedManager.init();
    biasManager.init();
    buzzerManager.init();
    configManager.init();
    // escStartupScript.init(false);  // TODO: enable ESC startup script
    //gtcManager.init();
}