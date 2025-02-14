// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/control/controller.hpp"
#include <cmath>
#include "brains2/common/tracks.hpp"
#include "generated/nmpc_solver.h"

using namespace brains2::control;

Controller::Controller(const Controller::ModelParams& model_params,
                       const Controller::Limits& limits,
                       const Controller::CostParams& cost_params)
    : solver_fun_mem{nullptr},
      x_guess{0.0},
      u_guess{0.0},
      x0{0.0},
      kappa_cen{0.0},
      w_cen{0.0},
      model_params{
          model_params.dt,
          model_params.m,
          model_params.l_R,
          model_params.l_F,
          model_params.C_m0,
          model_params.C_r0,
          model_params.C_r1,
          model_params.C_r2,
          model_params.t_delta,
          model_params.t_tau,
      },
      limits{
          limits.v_x_max,
          limits.delta_max,
          limits.tau_max,
      },
      cost_params{
          cost_params.v_x_ref, cost_params.delta_s_ref, cost_params.q_s,       cost_params.q_n,
          cost_params.q_psi,   cost_params.q_v_x,       cost_params.q_v_y,     cost_params.q_omega,
          cost_params.q_delta, cost_params.q_tau,       cost_params.r_delta,   cost_params.r_tau,
          cost_params.q_s_f,   cost_params.q_n_f,       cost_params.q_psi_f,   cost_params.q_v_x_f,
          cost_params.q_v_y_f, cost_params.q_omega_f,   cost_params.q_delta_f, cost_params.q_tau_f,
      },
      x_opt{0.0},
      u_opt{0.0} {
    this->solver_fun_mem = casadi_alloc(nmpc_solver_functions());
    this->solver_fun_mem->arg[0] = this->x_guess.data();
    this->solver_fun_mem->arg[1] = this->u_guess.data();
    this->solver_fun_mem->arg[2] = this->x0.data();
    this->solver_fun_mem->arg[3] = this->kappa_cen.data();
    this->solver_fun_mem->arg[4] = this->w_cen.data();
    this->solver_fun_mem->arg[5] = this->model_params.data();
    this->solver_fun_mem->arg[6] = this->limits.data();
    this->solver_fun_mem->arg[7] = this->cost_params.data();
    this->solver_fun_mem->res[0] = this->x_opt.data();
    this->solver_fun_mem->res[1] = this->u_opt.data();
}

tl::expected<Controller::Control, Controller::ControllerError> Controller::compute_control(
    const Controller::State& current_state, const brains2::common::Track& track) {
    // Project current position
    auto [s, pos_proj] =
        track.project(current_state.X, current_state.Y, track.s_min(), track.length());

    // Convert to Frenet pose
    auto phi_proj = track.eval_phi(s);
    double n = -(current_state.X - pos_proj(0)) * std::sin(phi_proj) +
               (current_state.Y - pos_proj(1)) * std::cos(phi_proj),
           psi = current_state.phi - phi_proj;

    // Set current state
    this->x0[0] = s;
    this->x0[1] = n;
    this->x0[2] = psi;
    this->x0[3] = current_state.v_x;
    this->x0[4] = current_state.v_y;
    this->x0[5] = current_state.omega;
    this->x0[6] = current_state.delta;
    this->x0[7] = current_state.tau;

    // Construct initial guess
    // Call solver
    // Check solver status
    // Extract solution
    return tl::unexpected(Controller::ControllerError::MAX_ITER);
}
