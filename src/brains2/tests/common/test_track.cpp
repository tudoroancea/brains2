// Copyright 2025 Tudor Oancea, Mateo Berthet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <stdexcept>
#include "brains2/common/tracks.hpp"
#include "gtest/gtest.h"

TEST(TrackTest, Constructor) {
    // Test constructor with valid input
    std::vector<double> s = {0.0, 1.0, 2.0};
    std::vector<double> X = {0.0, 1.0, 2.0};
    std::vector<double> Y = {0.0, 0.0, 0.0};
    std::vector<double> phi = {0.0, 0.0, 0.0};
    std::vector<double> kappa = {0.0, 0.0, 0.0};
    std::vector<double> width = {1.0, 1.0, 1.0};
    EXPECT_TRUE(brains2::common::Track::from_values(s, X, Y, phi, kappa, width).has_value());

    // Test constructor with invalid input
    std::vector<double> invalid_s = {0.0, 1.0};
    EXPECT_FALSE(
        brains2::common::Track::from_values(invalid_s, X, Y, phi, kappa, width).has_value());
}
