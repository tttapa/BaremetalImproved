#pragma once
#include <BaremetalCommunicationDef.hpp>  ///< FlightMode, WPTMode
#include <real_t.h>

/* Acceleration measurement in g. */
struct AccelMeasurement {
    AccelMeasurement(real_t ax, real_t ay, real_t az)
        : ax{ax}, ay{ay}, az{az} {}
    real_t ax;  ///< Acceleration along the x-axis in g.
    real_t ay;  ///< Acceleration along the y-axis in g.
    real_t az;  ///< Acceleration along the z-axis in g.
};

/* Angular velocity measurement in rad/s. */
struct GyroMeasurement {
    GyroMeasurement(real_t gx, real_t gy, real_t gz) : gx{gx}, gy{gy}, gz{gz} {}
    real_t gx;  ///< Angular velocity about the x-axis in rad/s.
    real_t gy;  ///< Angular velocity about the y-axis in rad/s.
    real_t gz;  ///< Angular velocity about the z-axis in rad/s.
};

/* Measured IMU angular velocity (gx,gy,gz) and acceleration (ax,ay,az). */
struct IMUMeasurement {
    IMUMeasurement(real_t gx, real_t gy, real_t gz, real_t ax, real_t ay,
                   real_t az)
        : gx{gx}, gy{gy}, gz{gz}, ax{ax}, ay{ay}, az{az} {}
    IMUMeasurement() = default;
    real_t gx;  ///< Acceleration along the x-axis in g.
    real_t gy;  ///< Acceleration along the y-axis in g.
    real_t gz;  ///< Acceleration along the z-axis in g.
    real_t ax;  ///< Angular velocity about the x-axis in rad/s.
    real_t ay;  ///< Angular velocity about the y-axis in rad/s.
    real_t az;  ///< Angular velocity about the z-axis in rad/s.
};

/**
 * Struct containing the values from the RC transmitter. This includes the
 * value of the throttle, roll, pitch and yaw, which range from 0 to 1. It
 * also contains the value of the tuner knob, which ranges from -0.5 to +0.5.
 * Lastly there are switches for the flight mode and the wireless power
 * transfer. These are represented by their respective enumerations.
 */
struct RCInput {
    RCInput(real_t throttle, real_t roll, real_t pitch, real_t yaw,
            real_t tuner, FlightMode flightMode, WPTMode wptMode)
        : throttle{throttle}, roll{roll}, pitch{pitch}, yaw{yaw}, tuner{tuner},
          flightMode{flightMode}, wptMode{wptMode} {}
    RCInput() = default;
    real_t throttle;        ///< Value of the RC throttle in [0,1].
    real_t roll;            ///< Value of the RC roll in [0,1].
    real_t pitch;           ///< Value of the RC pitch in [0,1].
    real_t yaw;             ///< Value of the RC yaw in [0,1].
    real_t tuner;           ///< Value of the RC tuner knob in [-0.5,+0.5].
    FlightMode flightMode;  ///< Value of the RC flight mode (as a FlightMode).
    WPTMode wptMode;        ///< Value of the RC WPT mode (as a WPTMode).
};