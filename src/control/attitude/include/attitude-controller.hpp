// Original: Cleanup-Pieter/Code-Generators/Controllers/include/attitude-controller.h

// ! Siel: this should be a symbolic link, so if we change the MATLAB code,
// !       then this will update as well.
// !
// !       Syntax if working directory == .../attitude/include/
// !       Syntax Windows: mklink "..\relative\path\file.hpp" "shortcut.hpp"
// !       Syntax Linux: ln -s "../relative/path/file.hpp" "shortcut.hpp"
// !
// !       Problem: we're not in the same project, so we can't use relative path to Codegen
// !                we're editing on different computers, so we can't use absolute path to Codegen
// !
// !       Solution 1: Move Cleanup-Pieter to BaremetalImproved and use relative path
// !       Solution 2: Move BaremetalImproved to GitLab and use relative path
// !       Solution 3: Remove [...]-controller.hpp and just link it in Vivado SDK
// !
// !       This is Pieter's GitHub and it's Pieter's directory in GitLab, so it's his decision
// !

#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "real_t.h"
    
typedef real_t ADD_CPP_REF(AttitudeOutput)[7];
typedef real_t ADD_CPP_REF(AttitudeControl)[3];
typedef real_t ADD_CPP_REF(AttitudeReference)[4];
typedef real_t ADD_CPP_REF(AttitudeState)[10];
typedef real_t ADD_CPP_REF(AttitudeIntegralWindup)[3];
typedef const real_t ADD_CPP_REF(const_AttitudeOutput)[7];
typedef const real_t ADD_CPP_REF(const_AttitudeControl)[3];
typedef const real_t ADD_CPP_REF(const_AttitudeReference)[4];
typedef const real_t ADD_CPP_REF(const_AttitudeState)[10];
typedef const real_t ADD_CPP_REF(const_AttitudeIntegralWindup)[3];


/**
 * @brief   Given the current state estimation `x_hat` and the target reference
 *          `ref`, calculate the control signal `u`.
 *
 * @param   x_hat
 *          An array containing the estimates of the ten attitude states.
 * @param   ref
 *          The reference quaternion as an array of four elements.
 * @param   u
 *          The output array where the three control vector will be stored.
 */
void getAttitudeControllerOutput(const_AttitudeState x_hat,
                                 const_AttitudeReference ref, 
                                 AttitudeControl u,
                                 AttitudeIntegralWindup y_int,
                                 int configuration,
                                 real_t tunerValue);
								 

void updateAttitudeKFEstimate(AttitudeState x_hat,
                              const_AttitudeControl u,
                              const_AttitudeOutput y,
                              int configuration);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_CONTROLLER_H */