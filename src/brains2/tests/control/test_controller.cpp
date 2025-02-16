#include "brains2/common/tracks.hpp"
#include "brains2/control/controller.hpp"
#include "gtest/gtest.h"

using namespace brains2::common;
using namespace brains2::control;

TEST(ControllerTestSuite, bruh) {
    std::vector<double> s = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> X = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> Y = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> phi = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> kappa = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> width = {3.0, 3.0, 3.0, 3.0};
    auto track = Track::from_values(s, X, Y, phi, kappa, width);
    ASSERT_TRUE(track.has_value());

    Controller controller(
        Controller::ModelParams{
            .dt = 0.05,
            .m = 230.0,
            .l_R = 0.7853,
            .l_F = 0.7853,
            .C_m0 = 4.950,
            .C_r0 = 350.0,
            .C_r1 = 20.0,
            .C_r2 = 3.0,
            .t_delta = 0.02,
            .t_tau = 1e-3,
        },
        Controller::Limits{
            .v_x_max = 10.0,
            .delta_max = 0.5,
            .tau_max = 100.0,
        },
        Controller::CostParams{.delta_s_ref = 0.25, .v_x_ref = 5.0, .q_s = 0.0});
    controller.compute_control(Controller::State{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, *track);
}
