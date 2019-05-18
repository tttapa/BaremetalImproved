#pragma once
#include <Square.hpp>
#include <MathFunctions.hpp>

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
        return Vec2f{std2::absf(v.x), std2::absf(v.y)};
    }
    static float normsq(const Vec2f &v) { return sq(v.x) + sq(v.y); }
    static float norm(const Vec2f &v) { return std2::sqrtf(normsq(v)); }
    static Vec2f round(const Vec2f &v) {
        return Vec2f{std2::roundf(v.x), std2::roundf(v.y)};
    }
    Vec2f abs() { return abs(*this); }
    float normsq() { return normsq(*this); }
    float norm() { return norm(*this); }
    Vec2f round() { return round(*this); }
} __attribute__((packed));

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
    static Vec3f abs(const Vec3f &v) {
        return Vec3f{std2::absf(v.x), std2::absf(v.y), std2::absf(v.z)};
    }
    static float normsq(const Vec3f &v) { return sq(v.x) + sq(v.y) + sq(v.z); }
    static float norm(const Vec3f &v) { return std2::sqrtf(normsq(v)); }
    static Vec3f round(const Vec3f &v) {
        return Vec3f{std2::roundf(v.x), std2::roundf(v.y), std2::roundf(v.z)};
    }
    Vec3f abs() { return abs(*this); }
    float normsq() { return normsq(*this); }
    float norm() { return norm(*this); }
    Vec3f round() { return round(*this); }
} __attribute__((packed));
