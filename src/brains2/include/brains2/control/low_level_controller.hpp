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

#ifndef BRAINS2_CONTROL_LOW_LEVEL_CONTROLLER_HPP
#define BRAINS2_CONTROL_LOW_LEVEL_CONTROLLER_HPP

#include <tuple>
#include "brains2/control/high_level_controller.hpp"
#include "brains2/sim/sim.hpp"

namespace brains2 {
namespace control {

class LowLevelController {
public:
    struct State {
        double v_x, v_y, omega, a_x, a_y, delta;
    };
    using Control = brains2::sim::Sim::Control;
    struct ModelParams {
        double m, l_R, l_F, axle_track, z_CG, C_downforce, torque_max;
    };
    struct Info {
        double omega_err, delta_tau;
    };

    LowLevelController() = delete;
    LowLevelController(const double K_tv, const ModelParams& model_params);

    std::pair<Control, Info> compute_control(const State& state, const HLC::Control& control);

private:
    double K_tv;
    ModelParams model_params;
};
typedef LowLevelController LLC;

}  // namespace control
}  // namespace brains2

#endif
