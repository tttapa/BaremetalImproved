#include <Position.hpp>

Position getCorrectedPosition(Position impMeasurement, float sonarMeasurement,
                              Quaternion orientation) {

    /* Calculate x- and y-component of the down vector using
       ealge-control-slides.pdf p.178. */
    Position downVectorXY = {
        -2 * (orientation.w * orientation.y + orientation.x * orientation.z),
        +2 * (orientation.w * orientation.x - orientation.y * orientation.z),
    };

    /* Correct the position of the drone. */
    return impMeasurement - downVectorXY * sonarMeasurement;
}

float getCorrectedHeight(float sonarMeasurement, Quaternion orientation) {

    /* Calculate the z-component of the down vector using
       eagle-control-slides.pdf p.178. */
    float downVectorZ =
        -orientation.w * orientation.w + orientation.x * orientation.x +
        orientation.y * orientation.y - orientation.z * orientation.z;

    /* Correct the height of the drone. */
    return -downVectorZ * sonarMeasurement;
}

Position getGlobalPositionEstimate(Position correctedPositionMeasurement,
                                   PositionState lastPositionEstimate,
                                   float Ts) {
    Position expected = lastPositionEstimate.p + lastPositionEstimate.v * Ts;
    Position delta    = expected - lastPositionEstimate.p;

    /* Calculate offset to be added to the given (x,y) using the expected
       location. E.g. expected (0, 0), measured (0.7, -1.1) should return
       an offset of (-2, +3) blocks thus a position of (0.1, -0.2). */
    Position offsetBlocks = (delta * METERS_TO_BLOCKS).round();
    return correctedPositionMeasurement + offsetBlocks * BLOCKS_TO_METERS;
}
