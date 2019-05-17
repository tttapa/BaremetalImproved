#pragma once
#include <Quaternion.hpp>

struct EulerAngles {
    real_t yaw;    ///< Z
    real_t pitch;  ///< Y'
    real_t roll;   ///< X"

    /** Convert a quaternion to Euler angles. */
    EulerAngles(Quaternion q = Quaternion::unit()) : EulerAngles{quat2eul(q)} {}
    EulerAngles(real_t yaw, real_t pitch, real_t roll)
        : yaw{yaw}, pitch{pitch}, roll{roll} {}

    /** 
     * Convert a quaternion to Euler angles.
     */
    static EulerAngles quat2eul(const Quaternion &q) {
        const real_t yaw   = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                                 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
        const real_t pitch = asin(2.0 * (q[0] * q[2] - q[3] * q[1]));
        const real_t roll  = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                                  1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        return EulerAngles{yaw, pitch, roll};
    }

    /** 
     * Convert Euler angles to a quaternion.
     */
    static Quaternion eul2quat(const EulerAngles &eul) {
        real_t cy = std::cos(eul.yaw / 2);
        real_t sy = std::sin(eul.yaw / 2);
        real_t cp = std::cos(eul.pitch / 2);
        real_t sp = std::sin(eul.pitch / 2);
        real_t cr = std::cos(eul.roll / 2);
        real_t sr = std::sin(eul.roll / 2);

        return {
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr,
        };
    }
};