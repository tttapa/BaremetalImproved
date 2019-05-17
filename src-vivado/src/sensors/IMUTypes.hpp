

/**
 * Frequency at which the gyroscope and the accelerometer of the IMU will run.
 */
enum IMUFrequency {

    /**
     * The gyroscope and accelerometer run at 119 Hz. The cutoff frequency of
     * the gyroscope low pass filter is set to 14 Hz by default. The bandwidth
     * of the accelerometer anti-aliasing filter is set to 50 Hz.
     */
    FREQ_119_HZ = 0,

    /**
     * The gyroscope and accelerometer run at 238 Hz. The cutoff frequency of
     * the gyroscope low pass filter is set to 14 Hz by default. The bandwidth
     * of the accelerometer anti-aliasing filter is set to 105 Hz.
     */
    FREQ_238_HZ = 1,

    /**
     * The gyroscope and accelerometer run at 476 Hz. The cutoff frequency of
     * the gyroscope low pass filter is set to 21 Hz by default. The bandwidth
     * of the accelerometer anti-aliasing filter is set to 211 Hz.
     */
    FREQ_476_HZ = 2,

    /**
     * The gyroscope and accelerometer run at 952 Hz. The cutoff frequency of
     * the gyroscope low pass filter is set to 33 Hz by default. The bandwidth
     * of the accelerometer anti-aliasing filter is set to 408 Hz.
     */
    FREQ_952_HZ = 3,
};

/** Maximum measurable angular velocity by the gyroscope in deg/s. */
enum GyroMaxSpeed {

    /** The maximum measurable angular velocity is ± 245 deg/s. */
    SPEED_245_DPS = 0,

    /** The maximum measurable angular velocity is ± 500 deg/s. */
    SPEED_500_DPS = 1,

    /** The maximum measurable angular velocity is ± 2000 deg/s. */
    SPEED_2000_DPS = 2,
};

/** Maximum measurable acceleration by the accelerometer in g. */
enum AccelMaxSpeed {

    /** The maximum measurable acceleration is ± 2 g. */
    SPEED_2_G = 0,

    /** The maximum measurable acceleration is ± 4 g. */
    SPEED_4_G = 1,

    /** The maximum measurable acceleration is ± 8 g. */
    SPEED_8_G = 2,

    /** The maximum measurable acceleration is ± 16 g. */
    SPEED_16_G = 3,
};

constexpr int getIMUBits(IMUFrequency frequency) {
    switch (frequency) {
        case FREQ_119_HZ: return 0b01100000;
        case FREQ_238_HZ: return 0b10000000;
        case FREQ_476_HZ: return 0b10100000;
        case FREQ_952_HZ: return 0b11000000;
        default: throw std::runtime_error(__PRETTY_FUNCTION__);
    }
}

constexpr int getIMUBits(GyroMaxSpeed maxSpeed) {
    switch (maxSpeed) {
        case SPEED_245_DPS: return 0b00000000;
        case SPEED_500_DPS: return 0b00001000;
        case SPEED_2000_DPS: return 0b00011000;
        default: throw std::runtime_error(__PRETTY_FUNCTION__);
    }
}

constexpr int getIMUBits(AccelMaxSpeed maxSpeed) {
    switch (maxSpeed) {
        case SPEED_2_G: return 0b00000000;
        case SPEED_4_G: return 0b00010000;
        case SPEED_8_G: return 0b00011000;
        case SPEED_16_G: return 0b00001000;
        default: throw std::runtime_error(__PRETTY_FUNCTION__);
    }
}

constexpr float getIMUValue(IMUFrequency frequency) {
    switch (frequency) {
        case FREQ_119_HZ: return 119.0;
        case FREQ_238_HZ: return 238.0;
        case FREQ_476_HZ: return 476.0;
        case FREQ_952_HZ: return 952.0;
        default: throw std::runtime_error(__PRETTY_FUNCTION__);
    }
}

constexpr float getIMUValue(GyroMaxSpeed maxSpeed) {
    switch (maxSpeed) {
        case SPEED_245_DPS: return 245.0;
        case SPEED_500_DPS: return 500.0;
        case SPEED_2000_DPS: return 2000.0;
        default: throw std::runtime_error(__PRETTY_FUNCTION__);
    }
}

constexpr float getIMUValue(AccelMaxSpeed maxSpeed) {
    switch (maxSpeed) {
        case SPEED_2_G: return 2.0;
        case SPEED_4_G: return 4.0;
        case SPEED_8_G: return 8.0;
        case SPEED_16_G: return 16.0;
        default: throw std::runtime_error(__PRETTY_FUNCTION__);
    }
}

constexpr int getIMUFactor(IMUFrequency frequency) {
    switch (frequency) {
        case FREQ_119_HZ: return 1;
        case FREQ_238_HZ: return 2;
        case FREQ_476_HZ: return 3;
        case FREQ_952_HZ: return 4;
        default: throw std::runtime_error(__PRETTY_FUNCTION__);
    }
}
