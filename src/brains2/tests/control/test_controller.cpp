#include <gtest/gtest-param-test.h>
#include <memory>
#include <numeric>
#include "brains2/common/tracks.hpp"
#include "brains2/control/controller.hpp"
#include "brains2/external/icecream.hpp"
#include "gtest/gtest.h"

using namespace brains2::common;
using namespace brains2::control;

static tl::optional<Track> generate_constant_curvature_track(const double curvature,
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

class ControllerConstantCurvatureTest : public ::testing::TestWithParam<double> {
protected:
    std::unique_ptr<Controller> controller;
    tl::optional<Track> track;
    void SetUp() override {
        controller = std::make_unique<Controller>(10,
                                                  Controller::ModelParams{
                                                       0.05,
                                                       230.0,
                                                       0.7853,
                                                       0.7853,
                                                       4.950,
                                                       350.0,
                                                       20.0,
                                                       3.0,
                                                  },
                                                  Controller::Limits{
                                                       10.0,
                                                       0.5,
                                                       100.0,
                                                  },
                                                  Controller::CostParams{
                                                       3.0,
                                                       5.0,
                                                       1.0,
                                                      1.0,
                                                      1.0,
                                                      1.0,
                                                      1.0,
                                                      1.0,
                                                      1.0,
                                                      1.0,
                                                      1.0,
                                                      1.0,
                                                  },
                                                  1);
        auto curvature = GetParam();
        track = generate_constant_curvature_track(curvature, 4.0, 5);
    }
};

TEST_P(ControllerConstantCurvatureTest, bruh) {
    ASSERT_TRUE(track.has_value());
    auto res = controller->compute_control(Controller::State{0.0, 0.0, 0.0, 0.0}, *track);
    ASSERT_TRUE(res.has_value());
    IC(*res, controller->get_x_opt(), controller->get_u_opt());
}

INSTANTIATE_TEST_SUITE_P(
    values,
    ControllerConstantCurvatureTest,
    testing::Values(-1 / 6.0, -1 / 8.0, -1 / 10.0, 0.0, 1 / 10.0, 1 / 8.0, 1 / 6.0));
