#pragma once
#include <Square.hpp>
#include <cmath>

/* Vector with two components, x and y, represented by floats. */
struct Vec2f {
    float x;
    float y;

    Vec2f(float x, float y) : x{x}, y{y} {}
    Vec2f() = default;

    inline Vec2f operator+(const Vec2f &rhs) const {
        return Vec2f{this->x + rhs.x, this->y + rhs.y};
    }
    inline Vec2f operator-(const Vec2f &rhs) const {
        return Vec2f{this->x - rhs.x, this->y - rhs.y};
    }
    inline Vec2f operator*(const float rhs) const {
        return Vec2f{this->x * rhs, this->y * rhs};
    }
    inline Vec2f operator/(const float rhs) const {
        return Vec2f{this->x / rhs, this->y / rhs};
    }

    static Vec2f abs(const Vec2f &v) {
        return Vec2f{fabs(v.x), fabs(v.y)};
    }
    static float normsq(const Vec2f &v) { return sq(v.x) + sq(v.y); }
    static float norm(const Vec2f &v) { return sqrt(normsq(v)); }
    static Vec2f round(const Vec2f &v) {
        return Vec2f{round(v.x), round(v.y)};
    }
    Vec2f abs() { return abs(*this); }
    float normsq() { return normsq(*this); }
    float norm() { return norm(*this); }
    Vec2f round() { return round(*this); }
};

/* Vector with three components, x, y and z, represented by floats. */
struct Vec3f {
    float x;
    float y;
    float z;

    Vec3f(float x, float y, float z) : x{x}, y{y}, z{z} {}
    Vec3f() = default;

    inline Vec3f operator+(const Vec3f &rhs) const {
        return Vec3f{this->x + rhs.x, this->y + rhs.y, this->z + rhs.z};
    }
    inline Vec3f operator-(const Vec3f &rhs) const {
        return Vec3f{this->x - rhs.x, this->y - rhs.y, this->z - rhs.z};
    }
    inline Vec3f operator*(const float rhs) const {
        return Vec3f{this->x * rhs, this->y * rhs, this->z * rhs};
    }
    inline Vec3f operator/(const float rhs) const {
        return Vec3f{this->x / rhs, this->y / rhs, this->z / rhs};
    }
    static float normsq(const Vec3f &v) { return sq(v.x) + sq(v.y) + sq(v.z); }
    static float norm(const Vec3f &v) { return sqrt(normsq(v)); }
    static Vec3f round(const Vec3f &v) {
        return Vec3f{std::round(v.x), std::round(v.y), std::round(v.z)};
    }
    float normsq() { return normsq(*this); }
    float norm() { return norm(*this); }
    Vec3f round() { return round(*this); }
};