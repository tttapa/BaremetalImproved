#pragma once
#include <real_t.h>
#include <Attitude.hpp>

// TODO: comment
real_t getCommonThrust();

MotorDutyCycles getDutyCycles();

void setCommonThrust(real_t value);

void setDutyCycles(MotorDutyCycles value);