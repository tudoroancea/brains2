#include "brains2/common/tracks.hpp"
#include "brains2/control/controller.hpp"
#include "brains2/external/icecream.hpp"
#include "gtest/gtest.h"

using namespace brains2::common;
using namespace brains2::control;

TEST(ControllerTestSuite, straight_line_test) {
    // 4m of straight line along the x axis,
    // 3m
    std::vector<double> s = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> X = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> Y = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> phi = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> kappa = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> width = {3.0, 3.0, 3.0, 3.0, 3.0};
    auto track = Track::from_values(s, X, Y, phi, kappa, width);
    ASSERT_TRUE(track.has_value());

    Controller controller(10,
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
                              .t_tau = 0.01,
                          },
                          Controller::Limits{
                              .v_x_max = 10.0,
                              .delta_max = 0.5,
                              .tau_max = 100.0,
                          },
                          Controller::CostParams{.delta_s_ref = 3.0,
                                                 .v_x_ref = 5.0,
                                                 .q_s = 1.0,
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
                                                 1.0,
                                                 1.0,
                                                 1.0,
                                                 1.0,
                                                 1.0,
                                                 1.0,
                                                 1.0},
                          10);
    auto res = controller.compute_control(Controller::State{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                          *track);
    ASSERT_TRUE(res.has_value());
    IC(*res, controller.get_x_opt(), controller.get_u_opt());
}
