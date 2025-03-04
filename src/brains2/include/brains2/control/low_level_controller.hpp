#ifndef BRAINS2_CONTROL_LOW_LEVEL_CONTROLLER_HPP
#define BRAINS2_CONTROL_LOW_LEVEL_CONTROLLER_HPP

#include "brains2/control/high_level_controller.hpp"
#include "brains2/sim/sim.hpp"

namespace brains2 {
namespace control {

class LowLevelController {
public:
    struct State {
        double v_x, v_y, omega, a_x, a_y;
    };
    using Control = brains2::sim::Sim::Control;
    struct ModelParams {
        double m, l_R, l_F, axle_track, z_CG, C_downforce, torque_max;
    };

    LowLevelController() = delete;
    LowLevelController(const double K_tv, const ModelParams& model_params);

    Control compute_control(const State& state, const HLC::Control& control);

private:
    double K_tv;
    ModelParams model_params;
};
typedef LowLevelController LLC;

}  // namespace control
}  // namespace brains2

#endif
