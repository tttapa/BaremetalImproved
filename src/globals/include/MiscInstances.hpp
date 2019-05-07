#pragma once
#include <BuzzerManager.hpp>
#include <ConfigurationManager.hpp>
#include <ESCStartupScript.hpp>
#include <GradualThrustChangeManager.hpp>
#include <RCManager.hpp>

/** Instance of the buzzer manager. */
extern BuzzerManager buzzerManager;

/** Instance of the configuration manager. */
extern ConfigurationManager configManager;

/** Instance of the ESC statup script. */
extern ESCStartupScript escStartupScript;

/** Instance of the gradual thrust change manager. */
extern GradualThrustChangeManager gtcManager;

/** Instance of the RC manager. */
extern RCManager rcManager;

/**
 * Initialize the miscellaneous instances.
 */
void initMiscInstances();
