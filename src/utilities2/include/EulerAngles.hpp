#pragma once
#include <Quaternion.hpp>

struct EulerAngles {
    float yaw;    ///< Z
    float pitch;  ///< Y'
    float roll;   ///< X"

    /** Convert a quaternion to Euler angles. */
    EulerAngles(Quaternion q = Quaternion::unit()) : EulerAngles{quat2eul(q)} {}
    EulerAngles(float yaw, float pitch, float roll)
        : yaw{yaw}, pitch{pitch}, roll{roll} {}

    /** 
     * Convert a quaternion to Euler angles.
     */
    static EulerAngles quat2eul(const Quaternion &q) {
        const float yaw   = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                                 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
        const float pitch = asin(2.0 * (q[0] * q[2] - q[3] * q[1]));
        const float roll  = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                                  1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        return EulerAngles{yaw, pitch, roll};
    }

    /** 
     * Convert Euler angles to a quaternion.
     */
    static Quaternion eul2quat(const EulerAngles &eul) {
        float cy = std::cos(eul.yaw / 2);
        float sy = std::sin(eul.yaw / 2);
        float cp = std::cos(eul.pitch / 2);
        float sp = std::sin(eul.pitch / 2);
        float cr = std::cos(eul.roll / 2);
        float sr = std::sin(eul.roll / 2);

        return {
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr,
        };
    }
};