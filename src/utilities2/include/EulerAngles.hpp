#pragma once
#include <Quaternion.hpp>
#include <cmath>

struct EulerAngles {
    float yaw;    ///< Z
    float pitch;  ///< Y'
    float roll;   ///< X"

    /** Convert a quaternion to Euler angles. */
    EulerAngles(Quaternion q = Quaternion::identity()) : EulerAngles{quat2eul(q)} {}
    EulerAngles(float yaw, float pitch, float roll)
        : yaw{yaw}, pitch{pitch}, roll{roll} {}

    /** 
     * Convert a quaternion to Euler angles.
     */
    static EulerAngles quat2eul(const Quaternion &q) {
        const float phi = std2::atan2f(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        const float theta = std2::asinf( 2.0 * (q.w * q.y - q.z * q.x));
        const float psi = std2::atan2f(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return EulerAngles{psi, theta, phi};
    }

    /** 
     * Convert Euler angles to a quaternion.
     */
    static Quaternion eul2quat(const EulerAngles &eul) {
        float cy = std2::cosf(eul.yaw / 2.0);
        float sy = std2::sinf(eul.yaw / 2.0);
        float cp = std2::cosf(eul.pitch / 2.0);
        float sp = std2::sinf(eul.pitch / 2.0);
        float cr = std2::cosf(eul.roll / 2.0);
        float sr = std2::sinf(eul.roll / 2.0);

        return {
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr,
        };
    }
};