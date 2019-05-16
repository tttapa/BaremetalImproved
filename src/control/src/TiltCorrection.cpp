#include <Position.hpp>

Position getCorrectedPosition(Position impMeasurement, real_t sonarMeasurement,
                              Quaternion orientation) {

    /* Calculate x- and y-component of the down vector using
       ealge-control-slides.pdf p.178. */
    Position downVectorXY = {
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

Position getGlobalPositionEstimate(Position correctedPositionMeasurement,
                                   PositionState lastPositionEstimate,
                                   real_t Ts) {
    Position expected = lastPositionEstimate.p + lastPositionEstimate.v * Ts;
    Position delta    = expected - lastPositionEstimate.p;

    /* Calculate offset to be added to the given (x,y) using the expected
       location. E.g. expected (0, 0), measured (0.7, -1.1) should return
       (-0.6, +0.9). */
    Position offsetBlocks = round(delta * METERS_TO_BLOCKS);
    return correctedPositionMeasurement + offsetBlocks * BLOCKS_TO_METERS;
}
