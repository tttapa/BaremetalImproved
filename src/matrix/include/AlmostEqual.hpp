#pragma once

#include "Array.hpp"
#include <cmath>

constexpr bool isAlmostEqual(double valuelhs, double valuerhs, double epsilon) {
    return std::abs(valuelhs - valuerhs) < epsilon;
}

template <class T, size_t N, class U>
constexpr bool isAlmostEqual(const Array<T, N> &arraylhs,
                             const Array<T, N> &arrayrhs, U epsilon) {
    for (size_t i = 0; i < N; ++i)
        if (!isAlmostEqual(arraylhs[i], arrayrhs[i], epsilon))
            return false;
    return true;
}