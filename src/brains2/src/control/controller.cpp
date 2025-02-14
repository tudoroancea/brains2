// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/control/controller.hpp"
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
    const Controller::State& current_state, const Controller::TrackParams& track_params) {
    // Copy track params and current state from velocity
    this->x0[3] = current_state.v_x;
    this->x0[4] = current_state.v_y;
    this->x0[5] = current_state.omega;
    this->x0[6] = current_state.delta;
    this->x0[7] = current_state.tau;
    std::copy(track_params.kappa_cen.begin(),
              track_params.kappa_cen.end(),
              this->kappa_cen.begin());
    std::copy(track_params.w_cen.begin(), track_params.w_cen.end(), this->w_cen.begin());

    // Project current position
    // Convert to Frenet pose
    // Construct initial guess
    // Call solver
    // Check solver status
    // Extract solution
    return tl::unexpected(Controller::ControllerError::MAX_ITER);
}
