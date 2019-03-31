#include <TiltCorrection.hpp>

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

ColVector<2> getCorrectedPosition(ColVector<2> locationMeasurement,
                                  real_t heightMeasurement, Quaternion q) {
    // Calculate down_vector with simplified quaternion rotation
    // because v = (0,0,-1)
    // (slide 178)
    ColVector<2> down_vector = {
        -2 * (q[0] * q[2] + q[1] * q[3]),
        +2 * (q[0] * q[1] - q[2] * q[3]),
    };
    // Calculate the position of the drone
    return locationMeasurement - heightMeasurement * down_vector;
}