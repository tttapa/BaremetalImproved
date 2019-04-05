#pragma once

#include <Matrix.hpp>
#include <Square.hpp>

class Quaternion {
  private:
    ColVector<4> q;

  public:
    constexpr Quaternion() : Quaternion{unit()} {}
    constexpr Quaternion(const ColVector<4> &q) : q{q} {}
    constexpr Quaternion(real_t q0, real_t q1, real_t q2, real_t q3)
        : q{q0, q1, q2, q3} {}

    constexpr ColVector<4> &asColVector() { return q; }
    constexpr const ColVector<4> &asColVector() const { return q; }

    constexpr Quaternion normalize() const { return {q / norm(q)}; }

    constexpr Quaternion conjugate() const {
        return {{q[0], -q[1], -q[2], -q[3]}};
    }

    constexpr Quaternion operator-() const { return conjugate(); }

    constexpr static Quaternion quatmultiply(const Quaternion &q,
                                             const Quaternion &r) {
        return {
            r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3],
            r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2],
            r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1],
            r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0],
        };
    }

    constexpr Quaternion operator+(const Quaternion &r) const {
        return quatmultiply(*this, r);
    }

    constexpr static Quaternion quatdifference(const Quaternion &q,
                                               const Quaternion &r) {
        return q + (-r);
    }

    constexpr Quaternion operator-(const Quaternion &r) const {
        return quatdifference(*this, r);
    }

    template <size_t C>
    constexpr Matrix<3, C> rotate(const Matrix<3, C> &v) {
        Matrix<3, 3> M = {{
            {
                1 - 2 * sq(q[2]) - 2 * sq(q[3]),
                2 * (q[1] * q[2] + q[0] * q[3]),
                2 * (q[1] * q[3] - q[0] * q[2]),
            },
            {
                2 * (q[1] * q[2] - q[0] * q[3]),
                1 - 2 * sq(q[1]) - 2 * sq(q[3]),
                2 * (q[2] * q[3] + q[0] * q[1]),
            },
            {
                2 * (q[1] * q[3] + q[0] * q[2]),
                2 * (q[2] * q[3] - q[0] * q[1]),
                1 - 2 * sq(q[1]) - 2 * sq(q[2]),
            },
        }};
        return M * v;
    }

    constexpr real_t &operator[](size_t index) { return q[index]; }
    constexpr const real_t &operator[](size_t index) const { return q[index]; }

    constexpr bool operator==(const Quaternion &other) const {
        return this->q == other.q;
    }

    constexpr bool operator!=(const Quaternion &other) const {
        return this->q != other.q;
    }

    constexpr operator ColVector<4> &() { return q; }
    constexpr operator const ColVector<4> &() const { return q; }

    constexpr static Quaternion unit() { return {1, 0, 0, 0}; }

    constexpr static Quaternion quatFromVec(ColVector<3> vec) {
    	real_t d = norm(vec);
    	real_t c = cos(d/2);
    	real_t s = d == 0 ? 0.0 : sin(d/2)/d;
    	return { c, s*vec[0], s*vec[1], s*vec[2] };
    }
};
