#include <TiltCorrection.hpp>

ColVector<2> getCorrectedPosition(ColVector<2> impMeasurement,
                                  real_t sonarMeasurement,
                                  Quaternion orientation) {

    /* Calculate x- and y-component of the down vector using
       ealge-control-slides.pdf p.178. */
    ColVector<2> downVectorXY = {
        -2 *
            (orientation[0] * orientation[2] + orientation[1] * orientation[3]),
        +2 *
            (orientation[0] * orientation[1] - orientation[2] * orientation[3]),
    };

    /* Correct the position of the drone. */
    return impMeasurement - sonarMeasurement * downVectorXY;
}

real_t getCorrectedHeight(real_t sonarMeasurement, Quaternion orientation) {

    /* Calculate the z-component of the down vector using
       eagle-control-slides.pdf p.178. */
    real_t downVectorZ =
        -orientation[0] * orientation[0] + orientation[1] * orientation[1] +
        orientation[2] * orientation[2] - orientation[3] * orientation[3];

    /* Correct the height of the drone. */
    return -downVectorZ * sonarMeasurement;
}
