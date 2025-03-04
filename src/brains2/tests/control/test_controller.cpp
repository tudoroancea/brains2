#include <gtest/gtest-param-test.h>
#include <memory>
#include <numeric>
#include "brains2/common/tracks.hpp"
#include "brains2/control/high_level_controller.hpp"
#include "brains2/external/icecream.hpp"
#include "gtest/gtest.h"

using namespace brains2::common;
using namespace brains2::control;

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

class ControllerConstantCurvatureTest : public ::testing::TestWithParam<double> {
protected:
    std::unique_ptr<HighLevelController> controller;
    std::unique_ptr<Track> track;
    void SetUp() override {
        controller = std::make_unique<HighLevelController>(10,
                                                           HighLevelController::ModelParams{
                                                               0.05,
                                                               230.0,
                                                               0.7853,
                                                               0.7853,
                                                               4.950,
                                                               350.0,
                                                               20.0,
                                                               3.0,
                                                           },
                                                           HighLevelController::ConstraintsParams{
                                                               10.0,
                                                               0.5,
                                                               100.0,
                                                               1.55,
                                                           },
                                                           HighLevelController::CostParams{
                                                               3.0,
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
                                                           });
        auto curvature = GetParam();
        const auto track_expected = generate_constant_curvature_track(curvature, 4.0, 5);
        if (!track_expected) {
            FAIL() << "Track generation failed";
        }
        this->track = std::make_unique<Track>(track_expected.value());
    }
};

TEST_P(ControllerConstantCurvatureTest, bruh) {
    auto res = controller->compute_control(HighLevelController::State{0.0, 0.0, 0.0, 0.0}, *track);
    ASSERT_TRUE(res.has_value());
    // IC(*res, controller->get_x_opt(), controller->get_u_opt());
}

INSTANTIATE_TEST_SUITE_P(
    values,
    ControllerConstantCurvatureTest,
    testing::Values(-1 / 6.0, -1 / 8.0, -1 / 10.0, 0.0, 1 / 10.0, 1 / 8.0, 1 / 6.0));
