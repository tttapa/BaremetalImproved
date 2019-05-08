#include <gtest/gtest.h>

#include <AlmostEqual.hpp>
#include <LeastSquares.hpp>

using Matrices::T;
using std::cout;
using std::endl;

static constexpr real_t eps = 1e3 * std::numeric_limits<real_t>::epsilon();

TEST(LeastSquares, solveLeastSquaresSquare) {
    Matrix<3, 3> A = {{
        {1, 1, 1},
        {0, 2, 5},
        {2, 5, -1},
    }};
    ColVector<3> B = {{
        {6},
        {-4},
        {27},
    }};

    ColVector<3> X = solveLeastSquares(A, B);

    ColVector<3> X_expected = {{
        {5},
        {3},
        {-2},
    }};

    ASSERT_TRUE(isAlmostEqual(X, X_expected, eps));
}