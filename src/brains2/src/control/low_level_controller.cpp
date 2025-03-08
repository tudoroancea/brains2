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

#include "brains2/control/low_level_controller.hpp"
#include "brains2/common/math.hpp"

using namespace brains2::control;
using namespace brains2::common;

LowLevelController::LowLevelController(const double K_tv, const ModelParams& model_params)
    : K_tv(K_tv), model_params(model_params) {
}

std::pair<LowLevelController::Control, LowLevelController::Info>
LowLevelController::compute_control(const State& state, const HLC::Control& control) {
    const auto& [m, l_R, l_F, axle_track, z_CG, C_downforce, torque_max] = model_params;
    const double wheelbase = l_F + l_R;

    // Compute reference torque difference between right and left wheels
    const auto beta = l_R / wheelbase * state.delta;
    const auto v = std::hypot(state.v_x, state.v_y);
    const double omega_kin = v * sin(beta) / l_R;
    const double delta_tau = K_tv * (omega_kin - state.omega);

    // Compute vertical loads
    const double F_downforce = 0.5 * C_downforce * state.v_x * state.v_x;
    const double static_weight = m * 9.81;
    const double front_weight_distribution = l_F / wheelbase;
    const double rear_weight_distribution = l_R / wheelbase;
    const double longitudinal_weight_transfer = m * state.a_x * z_CG / wheelbase;
    const double lateral_weight_transfer = m * state.a_y * z_CG / axle_track;
    const double F_z_FL = 0.5 * (front_weight_distribution * (static_weight + F_downforce) -
                                 longitudinal_weight_transfer - lateral_weight_transfer);
    const double F_z_FR = 0.5 * (front_weight_distribution * (static_weight + F_downforce) -
                                 longitudinal_weight_transfer + lateral_weight_transfer);
    const double F_z_RL = 0.5 * (rear_weight_distribution * (static_weight + F_downforce) +
                                 longitudinal_weight_transfer - lateral_weight_transfer);
    const double F_z_RR = 0.5 * (rear_weight_distribution * (static_weight + F_downforce) +
                                 longitudinal_weight_transfer + lateral_weight_transfer);

    // Adjust the torque of each wheel
    return std::make_pair(
        Control{control.u_delta,
                clip((control.tau - delta_tau) * F_z_FL / (static_weight + F_downforce),
                     -torque_max,
                     torque_max),
                clip((control.tau + delta_tau) * F_z_FR / (static_weight + F_downforce),
                     -torque_max,
                     torque_max),
                clip((control.tau - delta_tau) * F_z_RL / (static_weight + F_downforce),
                     -torque_max,
                     torque_max),
                clip((control.tau + delta_tau) * F_z_RR / (static_weight + F_downforce),
                     -torque_max,
                     torque_max)},
        Info{omega_kin - state.omega, delta_tau});
}
