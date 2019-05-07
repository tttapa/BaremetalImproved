#pragma once
#include <BuzzerManager.hpp>
#include <ConfigurationManager.hpp>
#include <ESCStartupScript.hpp>
#include <GradualThrustChange.hpp>

/** Instance of the buzzer manager. */
extern BuzzerManager buzzerManager;

/** Instance of the configuration manager. */
extern ConfigurationManager configManager;

/** Instance of the ESC statup script. */
extern ESCStartupScript escStartupScript;

/** Instance of the gradual thrust change manager. */
extern GradualThrustChangeManager gtcManager;

/**
 * Initialize the miscellaneous instances.
 */
void initMiscInstances();
