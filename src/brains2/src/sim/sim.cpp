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

#include "brains2/sim/sim.hpp"
#include <cmath>
#include "acados_c/sim_interface.h"
#include "brains2/common/math.hpp"
#include "generated/acados_sim_solver_dyn6.h"
#include "generated/acados_sim_solver_kin6.h"
#include "generated/dyn6_accels.h"
#include "generated/kin6_accels.h"

using brains2::common::clip;

namespace brains2::sim {

Sim::Sim(const Sim::Parameters &params, const Sim::Limits &limits)
    : limits(limits),
      x{0.0},
      u{0.0},
      p{params.m,
        params.I_z,
        params.l_R,
        params.l_F,
        params.C_m0,
        params.C_r0,
        params.C_r1,
        params.C_r2,
        params.t_T,
        params.t_delta,
        params.z_CG,
        params.axle_track,
        params.C_downforce,
        params.Ba,
        params.Ca,
        params.Da,
        params.Ea},
      x_next{0.0},
      a{0.0} {
    // Create the acados smulation solver
    kin6_workspace._sim_capsule = kin6_acados_sim_solver_create_capsule();
    auto kin6_capsule = reinterpret_cast<kin6_sim_solver_capsule *>(kin6_workspace._sim_capsule);
    kin6_acados_sim_create(kin6_capsule);
    kin6_workspace._sim_config = kin6_acados_get_sim_config(kin6_capsule);
    kin6_workspace._sim_in = kin6_acados_get_sim_in(kin6_capsule);
    kin6_workspace._sim_out = kin6_acados_get_sim_out(kin6_capsule);
    kin6_workspace._sim_dims = kin6_acados_get_sim_dims(kin6_capsule);

    // Update simulation/sampling time
    kin6_acados_sim_update_params(kin6_capsule, p.data(), Parameters::dim_kin6);

    // Allocate memory for the acceleration function
    kin6_workspace.accel_fun_mem = casadi_alloc(kin6_accels_functions());
    kin6_workspace.accel_fun_mem->arg[0] =
        x_next.data();  // accels will always be evaluated at the next state
    kin6_workspace.accel_fun_mem->arg[1] = p.data();
    kin6_workspace.accel_fun_mem->res[0] = a.data();

    // Create the acados smulation solver
    dyn6_workspace._sim_capsule = dyn6_acados_sim_solver_create_capsule();
    auto dyn6_capsule = reinterpret_cast<dyn6_sim_solver_capsule *>(dyn6_workspace._sim_capsule);
    dyn6_acados_sim_create(dyn6_capsule);
    dyn6_workspace._sim_config = dyn6_acados_get_sim_config(dyn6_capsule);
    dyn6_workspace._sim_in = dyn6_acados_get_sim_in(dyn6_capsule);
    dyn6_workspace._sim_out = dyn6_acados_get_sim_out(dyn6_capsule);
    dyn6_workspace._sim_dims = dyn6_acados_get_sim_dims(dyn6_capsule);

    // Update model parameters
    dyn6_acados_sim_update_params(dyn6_capsule, p.data(), Parameters::dim_dyn6);

    // Allocate memory for the acceleration function
    dyn6_workspace.accel_fun_mem = casadi_alloc(dyn6_accels_functions());
    dyn6_workspace.accel_fun_mem->arg[0] =
        x_next.data();  // accels will always be evaluated at the next state
    dyn6_workspace.accel_fun_mem->arg[1] = p.data();
    dyn6_workspace.accel_fun_mem->res[0] = a.data();
}

Sim::~Sim() {
    auto kin6_capsule = reinterpret_cast<kin6_sim_solver_capsule *>(kin6_workspace._sim_capsule);
    kin6_acados_sim_free(kin6_capsule);
    kin6_acados_sim_solver_free_capsule(kin6_capsule);
    auto dyn6_capsule = reinterpret_cast<dyn6_sim_solver_capsule *>(dyn6_workspace._sim_capsule);
    dyn6_acados_sim_free(dyn6_capsule);
    dyn6_acados_sim_solver_free_capsule(dyn6_capsule);
}

tl::expected<std::pair<Sim::State, Sim::Accels>, Sim::Error> Sim::simulate(
    const Sim::State &state, const Sim::Control &control, double dt) {
    // Set current state
    x[0] = state.X;
    x[1] = state.Y;
    x[2] = state.phi;
    x[3] = state.v_x;
    x[4] = state.v_y;
    x[5] = state.omega;
    x[6] = state.delta;
    x[7] = state.tau_FL;
    x[8] = state.tau_FR;
    x[9] = state.tau_RL;
    x[10] = state.tau_RR;

    // Set current target controls (and enforce limits)
    const double ddelta_max = p[Sim::Parameters::dim_kin6 - 1] * limits.delta_dot_max;
    u[0] = clip(clip(control.u_delta, state.delta - ddelta_max, state.delta + ddelta_max),
                -limits.delta_max,
                limits.delta_max);
    u[1] = clip(control.u_tau_FL, -limits.tau_max, limits.tau_max);
    u[2] = clip(control.u_tau_FR, -limits.tau_max, limits.tau_max);
    u[3] = clip(control.u_tau_RL, -limits.tau_max, limits.tau_max);
    u[4] = clip(control.u_tau_RR, -limits.tau_max, limits.tau_max);

    // Prohibit driving backwards by setting the control to zero if the car is going backwards.
    // This will let the car naturally coast to a stop, but we cannot accelerate backwards.
    if (state.v_x < 0.0) {
        u[1] = std::max(u[1], 0.0);
        u[2] = std::max(u[2], 0.0);
        u[3] = std::max(u[3], 0.0);
        u[4] = std::max(u[4], 0.0);
    }
    // depending on the last velocity v=sqrt(v_x^2+v_y^2), decide which model to
    // use and set its inputs.
    if (std::hypot(state.v_x, state.v_y) > 0.1) {
        // if (true) {
        // We use the dynamic model
        // Set simulation solver inputs
        sim_in_set(dyn6_workspace._sim_config,
                   dyn6_workspace._sim_dims,
                   dyn6_workspace._sim_in,
                   "x",
                   x.data());
        sim_in_set(dyn6_workspace._sim_config,
                   dyn6_workspace._sim_dims,
                   dyn6_workspace._sim_in,
                   "u",
                   u.data());

        // Update model parameters
        int exit_code = sim_in_set(dyn6_workspace._sim_config,
                                   dyn6_workspace._sim_dims,
                                   dyn6_workspace._sim_in,
                                   "T",
                                   &dt);
        if (exit_code != 0) {
            return tl::make_unexpected(Error::SAMPLING_TIME_UPDATE_ERROR);
        }

        // Call simulation solver
        exit_code = dyn6_acados_sim_solve(
            reinterpret_cast<dyn6_sim_solver_capsule *>(dyn6_workspace._sim_capsule));
        if (exit_code != 0) {
            return tl::make_unexpected(Error::ACADOS_SOLVER_ERROR);
        }

        // Get next state
        sim_out_get(dyn6_workspace._sim_config,
                    dyn6_workspace._sim_dims,
                    dyn6_workspace._sim_out,
                    "xn",
                    x_next.data());

        exit_code = casadi_eval(dyn6_workspace.accel_fun_mem);
        if (exit_code != 0) {
            return tl::make_unexpected(Error::ACCELS_FUNCTION_ERROR);
        }
    } else {
        // We use the kinematic model
        // Set simulation solver inputs
        sim_in_set(kin6_workspace._sim_config,
                   kin6_workspace._sim_dims,
                   kin6_workspace._sim_in,
                   "x",
                   x.data());
        sim_in_set(kin6_workspace._sim_config,
                   kin6_workspace._sim_dims,
                   kin6_workspace._sim_in,
                   "u",
                   u.data());

        int exit_code = sim_in_set(kin6_workspace._sim_config,
                                   kin6_workspace._sim_dims,
                                   kin6_workspace._sim_in,
                                   "T",
                                   &dt);
        if (exit_code != 0) {
            return tl::make_unexpected(Error::SAMPLING_TIME_UPDATE_ERROR);
        }

        // Call simulation solver
        exit_code = kin6_acados_sim_solve(
            reinterpret_cast<kin6_sim_solver_capsule *>(kin6_workspace._sim_capsule));
        if (exit_code != 0) {
            return tl::make_unexpected(Error::ACADOS_SOLVER_ERROR);
        }

        // Get next state
        sim_out_get(kin6_workspace._sim_config,
                    kin6_workspace._sim_dims,
                    kin6_workspace._sim_out,
                    "xn",
                    x_next.data());

        exit_code = casadi_eval(kin6_workspace.accel_fun_mem);
        if (exit_code != 0) {
            return tl::make_unexpected(Error::ACCELS_FUNCTION_ERROR);
        }
    }

    State next_state{x_next[0],
                     x_next[1],
                     x_next[2],
                     x_next[3],
                     x_next[4],
                     x_next[5],
                     x_next[6],
                     x_next[7],
                     x_next[8],
                     x_next[9],
                     x_next[10]};
    Accels next_accels{a[0], a[1]};

    // Check for NaNs
    if (std::any_of(x_next.begin(), x_next.end(), [](double value) { return std::isnan(value); }) ||
        std::any_of(a.begin(), a.end(), [](double value) { return std::isnan(value); })) {
        return tl::make_unexpected(Error::NANS_IN_RESULT);
    }

    return std::make_pair(next_state, next_accels);
}

std::string to_string(Sim::Error error) {
    switch (error) {
        case Sim::Error::SAMPLING_TIME_UPDATE_ERROR:
            return "SAMPLING_TIME_UPDATE_ERROR";
        case Sim::Error::ACADOS_SOLVER_ERROR:
            return "ACADOS_SOLVER_ERROR";
        case Sim::Error::ACCELS_FUNCTION_ERROR:
            return "ACCELS_FUNCTION_ERROR";
        case Sim::Error::NANS_IN_RESULT:
            return "NANS_IN_RESULT";
        default:
            return "UNKNOWN_ERROR";
    }
}

}  // namespace brains2::sim
