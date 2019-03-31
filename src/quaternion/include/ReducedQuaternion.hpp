#pragma once

#include "Quaternion.hpp"

using ReducedQuaternion = ColVector<3>;

// Reduced quaternion to full quaternion
template <size_t N>
constexpr ColVector<N + 1> red2quat(const ColVector<N> &r) {
    static_assert(
        N >= 3,
        "Error: reduced Quaternion should have at least three elements");
    ColVector<N + 1> qresult             = {};
    assignBlock<1, N + 1, 0, 1>(qresult) = r;
    qresult[0] = sqrt(1.0 - sq(r[0]) - sq(r[1]) - sq(r[2]));
    return qresult;
}

constexpr Quaternion red2quat(const ColVector<3> &r) {
    Quaternion qresult = {};
    qresult[1]         = r[0];
    qresult[2]         = r[1];
    qresult[3]         = r[2];
    qresult[0]         = sqrt(1.0 - sq(r[0]) - sq(r[1]) - sq(r[2]));
    return qresult;
}

// Quaternion to reduced quaternion
template <size_t N>
inline constexpr ColVector<N - 1> quat2red(const ColVector<N> &q) {
    return getBlock<1, N, 0, 1>(q);
}

constexpr ReducedQuaternion quat2red(const Quaternion &q) {
    return quat2red(q.asColVector());
}