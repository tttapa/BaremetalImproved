#include <AlmostEqual.hpp>
#include <Degrees.hpp>
#include <EulerAngles.hpp>
#include <TiltCorrection.hpp>
#include <gtest/gtest.h>

static constexpr real_t eps = 1e2 * std::numeric_limits<real_t>::epsilon();

// See eagle-control-slides.pdf: 182: Example (i)
#if 0
TEST(TiltCorrection, simpleFull) {
    real_t z_             = 1.101718;
    real_t theta          = 15_deg;
    real_t phi            = 20_deg;
    real_t psi            = 30_deg;
    Quaternion q          = EulerAngles{psi, theta, phi};
    ColVector<2> position = {42, 45};
    ColVector<3> result   = getCorrectedPositionFull(position, z_, q);
    ColVector<3> expected = {
        42 + 0.420455702855121,
        45 - 0.192352205961815,
        1.000000091161111,
    };
    EXPECT_TRUE(isAlmostEqual(result, expected, eps));
}
#endif

TEST(TiltCorrection, efficient) {
    real_t z_             = 1.101718;
    real_t theta          = 15_deg;
    real_t phi            = 20_deg;
    real_t psi            = 30_deg;
    Quaternion q          = EulerAngles{psi, theta, phi};
    ColVector<2> position = {42, 45};
    ColVector<2> result   = getCorrectedPosition(position, z_, q);
    ColVector<2> expected = {
        42 + 0.420455702855121,
        45 - 0.192352205961815,
    };

    EXPECT_TRUE(isAlmostEqual(result, expected, eps));
}