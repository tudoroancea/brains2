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

#ifndef BRAINS2__CONTROL__HIGH_LEVEL_CONTROLLER_HPP_
#define BRAINS2__CONTROL__HIGH_LEVEL_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "brains2/common/track.hpp"
#include "brains2/external/expected.hpp"
#include "casadi/casadi.hpp"
#include "Eigen/Dense"

namespace brains2 {
namespace control {

class HighLevelController {
public:
    struct ModelParams {
        double dt, m, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_delta;
    };
    struct CostParams {
        double v_ref, q_s, q_n, q_psi, q_v, r_delta, r_delta_dot, r_tau, q_s_f, q_n_f, q_psi_f,
            q_v_f;
    };
    struct ConstraintsParams {
        double v_max, delta_max, delta_dot_max, tau_max, car_width;
    };
    struct SolverParams {
        bool jit;
        std::string solver;
    };

    HighLevelController() = delete;
    HighLevelController(const HighLevelController&) = delete;
    HighLevelController& operator=(const HighLevelController&) = delete;
    virtual ~HighLevelController() = default;

    HighLevelController(size_t Nf,
                        const ModelParams& model_params,
                        const ConstraintsParams& limits,
                        const CostParams& cost_params,
                        const SolverParams& solver_params = {false, "fatrop"});

    struct State {
        double s, n, psi, v, delta;
    };
    struct Control {
        double u_delta, tau;
    };
    static constexpr uint8_t nx = sizeof(State) / sizeof(double);
    static constexpr uint8_t nu = sizeof(Control) / sizeof(double);
    typedef Eigen::Matrix<double, nx, Eigen::Dynamic> StateHorizonMatrix;
    typedef Eigen::Matrix<double, nu, Eigen::Dynamic> ControlHorizonMatrix;

    /*
     * @brief All the errors that may occur in the MPC.
     */
    enum class Error {
        MAX_ITER,
        NANS_IN_SOLVER,
        INFEASIBLE_PROBLEM,
        UNKNOWN_ERROR,
    };

    /*
     * @brief Crunch the numbers; solve the MPC.
     * @param current_state The current state of the vehicle.
     * @param track The current track estimate.
     * @return A control command or an error.
     */
    tl::expected<std::vector<Control>, Error> compute_control(const State& current_state,
                                                              const brains2::common::Track& track);

    /*
     * @brief Get the horizon size.
     */
    inline size_t horizon_size() const {
        return Nf;
    }

    /*
     * @brief Const ref getter to the optimal state trajectory. May contain outdated data if
     * compute_control() returned an error.
     */
    inline const StateHorizonMatrix& get_x_opt() const {
        return x_opt;
    }

    /*
     * @brief Const ref getter to the optimal control trajectory. May contain outdated data if
     * compute_control() returned an error.
     */
    inline const ControlHorizonMatrix& get_u_opt() const {
        return u_opt;
    }

    /*
     * @brief Const ref getter to the state reference trajectory.
     */
    inline const StateHorizonMatrix& get_x_ref() const {
        return x_ref;
    }

    /*
     * @brief Const ref getter to the control reference trajectory.
     */
    inline const ControlHorizonMatrix& get_u_ref() const {
        return u_ref;
    }

private:
    // cached controller parameters
    size_t Nf;
    double dt, v_ref, tau_ref;
    // reference state and control trajectories
    StateHorizonMatrix x_ref;
    ControlHorizonMatrix u_ref;
    // optimal state and control trajectories
    StateHorizonMatrix x_opt;
    ControlHorizonMatrix u_opt;
    // casadi symbolic variables for optimization variables and params
    casadi::Opti opti;
    casadi::MX cost_function;
    std::vector<casadi::MX> x, u;
    casadi::MX x0, kappa_cen, w_cen;
};

typedef HighLevelController HLC;

std::string to_string(const HLC::Error& error);

}  // namespace control
}  // namespace brains2

#endif  // BRAINS2__CONTROL__HIGH_LEVEL_CONTROLLER_HPP_
