#include <AlmostEqual.hpp>
#include <gtest/gtest.h>
#include <Matrix.hpp>

TEST(AlmostEqual, real_t) {
    real_t l = 1;
    real_t r = 1 + 1e-5;
    ASSERT_TRUE(isAlmostEqual(l, r, 1e-4));
    ASSERT_FALSE(isAlmostEqual(l, r, 1e-5));
}

TEST(AlmostEqual, matrix) {
    Matrix<2, 2> l = {{
        {1, 1},
        {1, 1},
    }};
    Matrix<2, 2> r = {{
        {1, 1},
        {1, 1 + 1e-5},
    }};
    ASSERT_TRUE(isAlmostEqual(l, r, 1e-4));
    ASSERT_FALSE(isAlmostEqual(l, r, 1e-5));
}