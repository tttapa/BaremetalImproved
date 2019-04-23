#include <TiltCorrection.hpp>

/* TODO: this should never be used, because the second function is simplified
ColVector<3> getCorrectedPositionFull(ColVector<2> rawPosition,
                                      real_t rawHeight,
                                      Quaternion orientation) {
    ColVector<3> v               = {{0, 0, -1}};
    Quaternion conjOrientation   = -orientation;
    ColVector<3> u               = conjOrientation.rotate(v);
    real_t height                = rawHeight * v * u;
    real_t horizontalDistanceAB  = sqrt(sq(rawHeight) - sq(height));
    ColVector<2> u_hat           = getBlock<0, 2, 0, 1>(u);
    ColVector<2> horizontalError = horizontalDistanceAB * normalize(u_hat);
    ColVector<3> position        = vcat(rawPosition - horizontalError, height);
    return position;
}
*/

ColVector<2> getCorrectedPosition(ColVector<2> locationMeasurement,
                                  real_t heightMeasurement, Quaternion q) {
    // Calculate down_vector with simplified quaternion rotation because v = (0,0,-1) (slide 178)
    ColVector<2> down_vector = {
        -2 * (q[0] * q[2] + q[1] * q[3]),
        +2 * (q[0] * q[1] - q[2] * q[3]),
    };
    // Calculate the position of the drone
    return locationMeasurement - heightMeasurement * down_vector;
}

real_t getCorrectedHeight(real_t heightMeasurement, Quaternion q) {

	// Calculate down vector: [0;down_vector] = q_drone (X) [0; 0; 0; -1] (X) q_drone^(-1)
	float down_vector_z = -q[0]*q[0] + q[1]*q[1] + q[2]*q[2] - q[3]*q[3];

	// Implement tilt correction: pz = down_vector' * [0; 0; 0; -1] * sonar
	return  -down_vector_z * sonar;
}
