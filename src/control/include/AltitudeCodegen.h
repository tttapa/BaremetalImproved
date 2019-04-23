#ifndef ALTITUDE_CODEGEN_H
#define ALTITUDE_CODEGEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <real_t.h>

/**
 * @brief   Given the current state estimation `x_hat` and the target reference
 *          `ref`, calculate the control signal `u`.
 *
 * @param   x_hat
 *          An array containing the estimates of the three altitude states.
 * @param   ref
 *          The reference altitude.
 * @param   u
 *          The output array where the control output will be stored.
 */
void getAltitudeControllerOutput(const_AltitudeState x_hat,
                                 const_AltitudeReference ref, 
                                 AltitudeControl u,
                                 AltitudeReference y_int,
                                 int configuration, real_t tunerValue);

void updateAltitudeKFEstimate(AltitudeState x_hat,
                              const_AltitudeControl u,
                              AltitudeOutput y,
                              int configuration);
/*void updateAltitudeObserver(AltitudeState x_hat,
                            const_AltitudeOutput y,
                            float altTs);*/

#ifdef __cplusplus
}
#endif

#endif /* ALTITUDE_CONTROLLER_H */
