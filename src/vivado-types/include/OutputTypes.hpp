#pragma once
#include <real_t.h>

/**
 * Instruction to be sent to the buzzer containing a duration (float), a buzzer
 * period (int) and a buzzer volume (int).
 */
struct BuzzerInstruction {
    BuzzerInstruction(real_t duration, int period, int volume)
        : duration{duration}, period{period}, volume{volume} {}
    real_t duration;  ///< Duration of instruction in seconds.
    int period;       ///< Period of sound, represented as an integer.
    int volume;       ///< Volume of sound, represented as an integer.
};

/**
 * Instruction to be sent to the LEDs, containing four booleans, representing
 * whether each of the four LEDs should be lit.
 */
struct LEDInstruction {
    LEDInstruction(bool led1, bool led2, bool led3, bool led4)
        : led1{led1}, led2{led2}, led3{led3}, led4{led4} {}
    bool led1;  ///< Whether the first LED should be lit.
    bool led2;  ///< Whether the second LED should be lit.
    bool led3;  ///< Whether the third LED should be lit.
    bool led4;  ///< Whether the fourth LED should be lit.
};

/**
 * Four floats representing the duty cycles to be sent to the four motors
 * (front-left, front-right, back-left, back-right). The four values should
 * be in [0, 1].
 */
struct MotorSignals {
    MotorSignals(real_t v0, real_t v1, real_t v2, real_t v3)
        : v0{v0}, v1{v1}, v2{v2}, v3{v3} {}
    real_t v0;  ///< Front-left motor duty cycle in [0,1].
    real_t v1;  ///< Front-right motor duty cycle in [0,1].
    real_t v2;  ///< Back-left motor duty cycle in [0,1].
    real_t v3;  ///< Back-right motor duty cycle in [0,1].
};
