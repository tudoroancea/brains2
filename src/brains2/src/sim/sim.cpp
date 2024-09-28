// Copyright (c) 2024, Tudor Oancea, Matteo Berthet
#include "brains2/sim/sim.hpp"
#include "brains2/common/math.hpp"
#include "brains2/external/icecream.hpp"
#include "models.h"

using namespace brains2::sim;
using namespace brains2::common;

Sim::Sim(const Sim::Parameters &params, const Sim::Limits &limits)
    : x{0.0},
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
      a{0.0},
      limits(limits),
      mem{nullptr} {
    mem = casadi_alloc(kin6_model_functions());
    mem->arg[0] = x.data();
    mem->arg[1] = u.data();
    mem->arg[2] = p.data();
    mem->res[0] = x_next.data();
    mem->res[1] = a.data();
}

Sim::~Sim() {
    casadi_free(mem);
}

std::pair<Sim::State, Sim::Accels> Sim::simulate(const Sim::State &state,
                                                 const Sim::Control &control,
                                                 double dt) {
    mem->arg[3] = &dt;
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
    const double ddelta_max = dt * limits.delta_dot_max;
    u[0] = clip(clip(control.u_delta, state.delta - ddelta_max, state.delta + ddelta_max),
                -limits.delta_max,
                limits.delta_max);
    u[1] = clip(control.u_tau_FL, -limits.tau_max, limits.tau_max);
    u[2] = clip(control.u_tau_FR, -limits.tau_max, limits.tau_max);
    u[3] = clip(control.u_tau_RL, -limits.tau_max, limits.tau_max);
    u[4] = clip(control.u_tau_RR, -limits.tau_max, limits.tau_max);

    int exit_code = casadi_eval(mem);

    // TODO(tudoroancea): use optionals
    if (exit_code != 0) {
        throw std::runtime_error("Simulation returned non-zero exit code");
    }
    if (std::isnan(x_next[0]) or std::isnan(x_next[1]) or std::isnan(x_next[2]) or
        std::isnan(x_next[3]) or std::isnan(x_next[4]) or std::isnan(x_next[5]) or
        std::isnan(x_next[6]) or std::isnan(x_next[7]) or std::isnan(x_next[8]) or
        std::isnan(x_next[9]) or std::isnan(x_next[10]) or std::isnan(a[0]) or std::isnan(a[1])) {
        throw std::runtime_error("Simulation returned NaN");
    }

    return {State{x_next[0],
                  x_next[1],
                  x_next[2],
                  x_next[3],
                  x_next[4],
                  x_next[5],
                  x_next[6],
                  x_next[7],
                  x_next[8],
                  x_next[9],
                  x_next[10]},
            Accels{a[0], a[1]}};
}
