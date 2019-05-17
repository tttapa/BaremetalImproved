#pragma once
#include <Square.hpp>
#include <Vector.hpp>

/** Struct representing a quaterniong. */
struct Quaternion {
    float w;
    float x;
    float y;
    float z;

    Quaternion() : Quaternion{unit()} {}
    Quaternion(float w, float x, float y, float z) : w{w}, x{x}, y{y}, z{z} {}

    static Quaternion conjugate(const Quaternion &q) {
        return {q.w, -q.x, -q.y, -q.z};
    }

    Quaternion operator-() const { return conjugate(*this); }

    static Quaternion quatmultiply(const Quaternion &lhs,
                                   const Quaternion &rhs) {
        return {
            rhs.w * lhs.w - rhs.x * lhs.x - rhs.y * lhs.y - rhs.z * lhs.z,
            rhs.w * lhs.x + rhs.x * lhs.w - rhs.y * lhs.z + rhs.z * lhs.y,
            rhs.w * lhs.y + rhs.x * lhs.z + rhs.y * lhs.w - rhs.z * lhs.x,
            rhs.w * lhs.z - rhs.x * lhs.y + rhs.y * lhs.x + rhs.z * lhs.w,
        };
    }

    Quaternion operator+(const Quaternion &rhs) const {
        return quatmultiply(*this, rhs);
    }

    static Quaternion quatdifference(const Quaternion &l, const Quaternion &r) {
        return l + (-r);
    }

    Quaternion operator-(const Quaternion &rhs) const {
        return quatdifference(*this, rhs);
    }

    Vec3f rotate(const Vec3f &v) {

        /* Rotation matrix. */
        float M11 = 1 - 2 * sq(this->y) - 2 * sq(this->z);
        float M12 = 2 * (this->x * this->y + this->w * this->z);
        float M13 = 2 * (this->x * this->z - this->w * this->y);
        float M21 = 2 * (this->x * this->y - this->w * this->z);
        float M22 = 1 - 2 * sq(this->x) - 2 * sq(this->z);
        float M23 = 2 * (this->y * this->z + this->w * this->x);
        float M31 = 2 * (this->x * this->z + this->w * this->y);
        float M32 = 2 * (this->y * this->z - this->w * this->x);
        float M33 = 1 - 2 * sq(this->x) - 2 * sq(this->y);

        return Vec3f{
            M11 * v.x + M12 * v.y + M13 * v.z,  //
            M21 * v.x + M22 * v.y + M23 * v.z,  //
            M31 * v.x + M32 * v.y + M33 * v.z,  //
        };
    }

    static Quaternion unit() { return Quaternion{1, 0, 0, 0}; }

    /**
     * Calculate the quaternion that would result in the given in vector, if it
     * were to used to rotate the vector (0 0 1).
     */
    static Quaternion fromDirection(Vec3f &v) {
        /*
    	 * q = cos(ϑ / 2) + sin(ϑ / 2)·(x·i + y·j + z·k)
    	 * where (x y z) is a unit vector representing the axis about which
    	 * the body is rotated; ϑ is the angle by which it is rotated.
    	 *
    	 * (x y z) is the cross product between a vector pointing upwards (0 0 1)
    	 * and the given vector; ϑ can be found using A×B = |A||B|·sin(ϑ).
    	 */

        /* First check the edge case v ~ (0 0 1). */
        float eps = std::numeric_limits<float>::epsilon();
        if (abs(v.x) <= eps && abs(v.y) <= eps)
            return Quaternion::unit();

        /* Calculate the cross product and its norm. */
        Vec3f cross     = {v.y, -v.x, 0};
        float crossNorm = cross.norm();
        cross.x /= crossNorm;
        cross.y /= crossNorm;

        /* Calculate the angle ϑ. */
        float angle = std::asin(crossNorm / v.norm());

        /* Calculate the resulting quaternion. */
        return Quaternion{std::cos(angle / 2), std::sin(angle / 2) * cross.x,
                          std::sin(angle / 2) * cross.y,
                          std::sin(angle / 2) * cross.z};
    }
};
