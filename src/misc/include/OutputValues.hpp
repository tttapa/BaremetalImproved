#pragma once
#include <real_t.h>
#include <Attitude.hpp>

// TODO: comment
real_t getCommonThrust();

MotorSignals getMotorSignals();

void setCommonThrust(real_t value);

void setMotorSignals(MotorSignals value);