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
#include "brains2/common/track.hpp"
#include "gtest/gtest.h"

TEST(TrackTestSuite, construction_straight_line) {
    // Test constructor with valid input
    std::vector<double> s{0.0, 1.0, 2.0};
    std::vector<double> X{0.0, 1.0, 2.0};
    std::vector<double> Y{0.0, 0.0, 0.0};
    std::vector<double> phi{0.0, 0.0, 0.0};
    std::vector<double> kappa{0.0, 0.0, 0.0};
    std::vector<double> width{1.0, 1.0, 1.0};
    EXPECT_TRUE(brains2::common::Track::from_values(s, X, Y, phi, kappa, width).has_value());
}

TEST(TrackTestSuite, construction_different_sizes) {
    // for each array of values used to construct a track,
    // try several ways to remove one element and check that
    // the construction fails
    std::vector<double> s{0.0, 1.0, 2.0};
    std::vector<double> X{0.0, 1.0, 2.0};
    std::vector<double> Y{0.0, 0.0, 0.0};
    std::vector<double> phi{0.0, 0.0, 0.0};
    std::vector<double> kappa{0.0, 0.0, 0.0};
    std::vector<double> width{1.0, 1.0, 1.0};
    const std::vector<std::function<void(std::vector<double>*)>> modifications{
        [](std::vector<double>* v) { v->pop_back(); },
        [](std::vector<double>* v) { v->erase(v->begin()); },
        [](std::vector<double>* v) { v->erase(std::next(v->begin())); },
    };
    for (auto* to_modify : {&s, &X, &Y, &phi, &kappa, &width}) {
        const std::vector<double> backup = *to_modify;
        for (const auto& modification : modifications) {
            modification(to_modify);
            EXPECT_FALSE(
                brains2::common::Track::from_values(s, X, Y, phi, kappa, width).has_value());
            *to_modify = backup;
        }
    }
}

TEST(TrackTestSuite, construction_nonmonotonic_progress) {
    std::vector<double> s{0.0, 1.0, 2.0, 3.0};
    std::vector<double> X{0.0, 1.0, 2.0, 3.0};
    std::vector<double> Y{0.0, 0.0, 0.0, 0.0};
    std::vector<double> phi{0.0, 0.0, 0.0, 0.0};
    std::vector<double> kappa{0.0, 0.0, 0.0, 0.0};
    std::vector<double> width{1.0, 1.0, 1.0, 1.0};
    constexpr double offset = 0.1;
    // Break monotonicity of s by changing the value of one value at a time
    // and check that the construction fails
    for (size_t i = 1; i < s.size(); ++i) {
        s[i] = s[i - 1] - offset;
        EXPECT_FALSE(brains2::common::Track::from_values(s, X, Y, phi, kappa, width).has_value());
        s[i] = s[i - 1] + offset;
    }
}

TEST(TrackTestSuite, construction_nonpositive_width) {
    std::vector<double> s{0.0, 1.0, 2.0, 3.0};
    std::vector<double> X{0.0, 1.0, 2.0, 3.0};
    std::vector<double> Y{0.0, 0.0, 0.0, 0.0};
    std::vector<double> phi{0.0, 0.0, 0.0, 0.0};
    std::vector<double> kappa{0.0, 0.0, 0.0, 0.0};
    std::vector<double> width{1.0, 1.0, 1.0, 1.0};
    // set one width value to a non-positive value and check that the construction fails
    for (double& w : width) {
        const double tpr = w;
        w *= -1;
        EXPECT_FALSE(brains2::common::Track::from_values(s, X, Y, phi, kappa, width).has_value());
        w = 0.0;
        EXPECT_FALSE(brains2::common::Track::from_values(s, X, Y, phi, kappa, width).has_value());
        w = tpr;
    }
}

TEST(TrackTestSuite, construction_discontinuous_heading) {
    std::vector<double> s{0.0, 1.0, 2.0, 3.0};
    std::vector<double> X{0.0, 1.0, 2.0, 3.0};
    std::vector<double> Y{0.0, 0.0, 0.0, 0.0};
    std::vector<double> phi{0.2, 0.1, 0.0, 2 * M_PI - 0.1};
    std::vector<double> kappa{0.0, 0.0, 0.0, 0.0};
    std::vector<double> width{1.0, 1.0, 1.0, 1.0};
    EXPECT_FALSE(brains2::common::Track::from_values(s, X, Y, phi, kappa, width).has_value());
}

// TEST(TrackTestSuite, project);
// TEST(TrackTestSuite, frenet_to_cartesian);
// TEST(TrackTestSuite, cartesian_to_frenet);
