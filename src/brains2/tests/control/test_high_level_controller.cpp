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

#include <memory>
#include <numeric>
#include "brains2/common/track.hpp"
#include "brains2/control/high_level_controller.hpp"
#include "brains2/external/icecream.hpp"
#include "Eigen/Dense"
#include "gtest/gtest.h"

using brains2::common::Track;
using brains2::control::HighLevelController;

static tl::expected<Track, Track::Error> generate_constant_curvature_track(const double curvature,
                                                                           const double s_max,
                                                                           const size_t N) {
    Eigen::VectorXd s = Eigen::VectorXd::LinSpaced(N, 0.0, s_max);
    Eigen::VectorXd kappa = curvature * Eigen::VectorXd::Ones(N);
    Eigen::VectorXd width = 1.5 * Eigen::VectorXd::Ones(N);
    if (std::fabs(curvature) < 1e-6) {
        return Track::from_values(s,
                                  s,
                                  Eigen::VectorXd::Zero(N),
                                  Eigen::VectorXd::Zero(N),
                                  kappa,
                                  width);
    }
    const auto R = 1 / std::abs(curvature);
    const Eigen::VectorXd angles = (s / R).array() - M_PI_2;
    const Eigen::VectorXd X = R * angles.array().cos();
    const Eigen::VectorXd Y = (R * angles.array().sin() + R) * (curvature > 0.0 ? 1.0 : -1.0);
    const Eigen::VectorXd phi = (angles.array() + M_PI_2) * (curvature > 0.0 ? 1.0 : -1.0);
    return Track::from_values(s, X, Y, phi, kappa, width);
}

class HighLevelControllerTestSuite : public ::testing::TestWithParam<double> {
protected:
    double curvature;
    std::unique_ptr<HighLevelController> controller;
    std::unique_ptr<Track> track;
    void SetUp() override {
        this->controller =
            std::make_unique<HighLevelController>(20,
                                                  HighLevelController::ModelParams{
                                                      0.05,
                                                      230.0,
                                                      0.7853,
                                                      0.7853,
                                                      4.950,
                                                      350.0,
                                                      20.0,
                                                      3.0,
                                                      0.02,
                                                  },
                                                  HighLevelController::ConstraintsParams{
                                                      10.0,
                                                      0.5,
                                                      1.0,
                                                      200.0,
                                                      1.55,
                                                  },
                                                  HighLevelController::CostParams{
                                                      3.0,
                                                      10.0,
                                                      20.0,
                                                      50.0,
                                                      20.0,
                                                      2.0,
                                                      1.0,
                                                      0.0001,
                                                      10000.0,
                                                      20000.0,
                                                      50000.0,
                                                      20000.0,
                                                  });
        curvature = GetParam();
        const auto track_expected = generate_constant_curvature_track(curvature, 4.0, 5);
        if (!track_expected) {
            FAIL() << "Track generation failed";
        }
        this->track = std::make_unique<Track>(track_expected.value());
    }
};

TEST_P(HighLevelControllerTestSuite, constant_curvature) {
    auto res =
        controller->compute_control(HighLevelController::State{0.0, 0.0, 0.0, 0.0, 0.0}, *track);
    ASSERT_TRUE(res.has_value());
    IC(curvature, controller->get_x_opt(), controller->get_u_opt());
}

INSTANTIATE_TEST_SUITE_P(
    ,
    HighLevelControllerTestSuite,
    testing::Values(-1 / 6.0, -1 / 8.0, -1 / 10.0, 0.0, 1 / 10.0, 1 / 8.0, 1 / 6.0));
