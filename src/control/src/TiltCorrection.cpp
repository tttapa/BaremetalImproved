#include <Position.hpp>

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

real_t sgn(real_t value) { return copysign(1.0, value); }

ColVector<2>
getGlobalPositionEstimate(ColVector<2> correctedPositionMeasurement,
                          PositionState lastPositionEstimate, real_t Ts) {
    real_t xExpected = lastPositionEstimate.p.x + lastPositionEstimate.vx * Ts;
    real_t yExpected = lastPositionEstimate.p.y + lastPositionEstimate.vy * Ts;
    real_t dx        = xExpected - lastPositionEstimate.p.x;
    real_t dy        = yExpected - lastPositionEstimate.p.y;

    /* Calculate offset to be added to the given (x,y) using the expected
       location. E.g. expected (0, 0), measured (0.7, -1.1) should return
       (-0.6, +0.9). */
    real_t offsetXBlocks = round(dx * METERS_TO_BLOCKS);
    real_t offsetYBlocks = round(dy * METERS_TO_BLOCKS);
    return correctedPositionMeasurement +
           ColVector<2>{offsetXBlocks * BLOCKS_TO_METERS,
                        offsetYBlocks * BLOCKS_TO_METERS};
}
