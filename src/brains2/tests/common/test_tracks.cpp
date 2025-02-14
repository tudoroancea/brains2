#include <stdexcept>
#include "brains2/common/tracks.hpp"
#include "gtest/gtest.h"

TEST(TracksTest, Constructor) {
    // Test constructor with valid input
    std::vector<double> s = {0.0, 1.0, 2.0};
    std::vector<double> X = {0.0, 1.0, 2.0};
    std::vector<double> Y = {0.0, 0.0, 0.0};
    std::vector<double> phi = {0.0, 0.0, 0.0};
    std::vector<double> kappa = {0.0, 0.0, 0.0};
    std::vector<double> width = {1.0, 1.0, 1.0};
    EXPECT_NO_THROW(brains2::common::Track(s, X, Y, phi, kappa, width));

    // Test constructor with invalid input
    std::vector<double> invalid_s = {0.0, 1.0};
    EXPECT_THROW(brains2::common::Track(invalid_s, X, Y, phi, kappa, width), std::invalid_argument);
}
