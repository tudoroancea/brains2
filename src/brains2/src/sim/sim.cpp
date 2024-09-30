// Copyright (c) 2024, Tudor Oancea, Matteo Berthet
#include "brains2/sim/sim.hpp"
#include "acados_c/sim_interface.h"
#include "brains2/common/math.hpp"
#include "generated/acados_sim_solver_kin6.h"
#include "generated/kin6_accels.h"

using namespace brains2::sim;
using namespace brains2::common;

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
        params.t_delta},
      x_next{0.0},
      a{0.0} {
    // Create the acados smulation solver
    kin6_sim_capsule = kin6_acados_sim_solver_create_capsule();
    int status = kin6_acados_sim_create((kin6_sim_solver_capsule *)kin6_sim_capsule);
    if (status) {
        throw std::runtime_error("kin6_acados_sim_create() returned status " +
                                 std::to_string(status));
    }
    kin6_sim_config = kin6_acados_get_sim_config((kin6_sim_solver_capsule *)kin6_sim_capsule);
    kin6_sim_in = kin6_acados_get_sim_in((kin6_sim_solver_capsule *)kin6_sim_capsule);
    kin6_sim_out = kin6_acados_get_sim_out((kin6_sim_solver_capsule *)kin6_sim_capsule);
    kin6_sim_dims = kin6_acados_get_sim_dims((kin6_sim_solver_capsule *)kin6_sim_capsule);

    // Allocate memory for the acceleration function
    accel_fun_mem = casadi_alloc(accels_functions());
    accel_fun_mem->arg[0] = x_next.data();  // accels will always be evaluated at the next state
    accel_fun_mem->arg[1] = p.data();
    accel_fun_mem->res[0] = a.data();
}

Sim::~Sim() {
    // TODO: what happens if the free fails?
    kin6_acados_sim_free((kin6_sim_solver_capsule *)kin6_sim_capsule);
    kin6_acados_sim_solver_free_capsule((kin6_sim_solver_capsule *)kin6_sim_capsule);
}

std::pair<Sim::State, Sim::Accels> Sim::simulate(const Sim::State &state,
                                                 const Sim::Control &control,
                                                 double dt) {
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
    const double ddelta_max = dt * limits.delta_dot_max;
    u[0] = clip(clip(control.u_delta, state.delta - ddelta_max, state.delta + ddelta_max),
                -limits.delta_max,
                limits.delta_max);
    u[1] = clip(control.u_tau_FL, -limits.tau_max, limits.tau_max);
    u[2] = clip(control.u_tau_FR, -limits.tau_max, limits.tau_max);
    u[3] = clip(control.u_tau_RL, -limits.tau_max, limits.tau_max);
    u[4] = clip(control.u_tau_RR, -limits.tau_max, limits.tau_max);

    // TODO(#25): add switch to dynamic model
    // depending on the last velocity v=sqrt(v_x^2+v_y^2), decide which model to
    // use and set its inputs.

    // Set simulation solver inputs
    sim_in_set(kin6_sim_config, kin6_sim_dims, kin6_sim_in, "x", x.data());
    sim_in_set(kin6_sim_config, kin6_sim_dims, kin6_sim_in, "u", u.data());

    // TODO(tudoroancea): use optionals
    // Update simulation/sampling time
    int exit_code = kin6_acados_sim_update_params((kin6_sim_solver_capsule *)kin6_sim_capsule,
                                                  p.data(),
                                                  p.size());
    if (exit_code != 0) {
        throw std::runtime_error("Failed to update parameters");
    }

    // Call simmulation solver
    sim_in_set(kin6_sim_config, kin6_sim_dims, kin6_sim_in, "T", &dt);
    exit_code = kin6_acados_sim_solve((kin6_sim_solver_capsule *)kin6_sim_capsule);
    if (exit_code != 0) {
        throw std::runtime_error("Simulation returned non-zero exit code");
    }

    // Get next state
    sim_out_get(kin6_sim_config, kin6_sim_dims, kin6_sim_out, "xn", x_next.data());

    exit_code = casadi_eval(accel_fun_mem);
    if (exit_code != 0) {
        throw std::runtime_error("Acceleration function returned non-zero exit code");
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
        throw std::runtime_error("Simulation returned NaN");
    }

    // Prohibit the car from going backwards
    if (next_state.v_x < 0.0 or
        (next_state.tau_FL <= 0.1 and next_state.tau_FR <= 0.1 and next_state.tau_RL <= 0.1 and
         next_state.tau_RR <= 0.1 and next_state.v_x <= 0.01)) {
        next_state.v_x = 0.0;
        next_state.v_y = 0.0;
        next_state.omega = 0.0;
    }
    return {next_state, next_accels};
}
