#include <real_t.h>
#include <attitude.hpp>

void tiltCorrection(real_t pz) {

    float down_vector_z;
    ColVector<4> state_quat = Attitude::x_hat.q
	down_vector_z = -Attitude::x_hat.q[0]
    
    
    1*att_hat[0] + att_hat[1]*att_hat[1] + att_hat[2]*att_hat[2] - att_hat[3]*att_hat[3];

	// Implement tilt correction: pz = down_vector' * [0; 0; 0; -1] * sonar
	pz = - down_vector_z * sonar;
	
	// Save corrected measurement in alt_y
	alt_y[0] = pz;

}

// TODO interactie met Attitude regelen

