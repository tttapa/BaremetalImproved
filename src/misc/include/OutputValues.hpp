#pragma once

/* Includes from src. */
#include <Attitude.hpp>
#include <real_t.h>

// TODO: comment
real_t getCommonThrust();

MotorSignals getMotorSignals();

void setCommonThrust(real_t value);

void setMotorSignals(MotorSignals value);