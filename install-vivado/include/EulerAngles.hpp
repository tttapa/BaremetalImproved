#pragma once
#include <Quaternion.hpp>
#include <cmath>

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
        const float yaw   = atan2(2.0 * (q.w * q.x + q.y * q.z),
                                 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        const float pitch = asin(2.0 * (q.w * q.y - q.z * q.x));
        const float roll  = atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return EulerAngles{yaw, pitch, roll};
    }

    /** 
     * Convert Euler angles to a quaternion.
     */
    static Quaternion eul2quat(const EulerAngles &eul) {
        float cy = cos(eul.yaw / 2);
        float sy = sin(eul.yaw / 2);
        float cp = cos(eul.pitch / 2);
        float sp = sin(eul.pitch / 2);
        float cr = cos(eul.roll / 2);
        float sr = sin(eul.roll / 2);

        return {
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr,
        };
    }
};