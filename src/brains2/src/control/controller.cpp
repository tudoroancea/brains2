// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/control/controller.hpp"
#include <cmath>
#include "brains2/common/tracks.hpp"
#include "brains2/external/expected.hpp"
#include "brains2/external/icecream.hpp"
#include "casadi/casadi.hpp"
#include "Eigen/Dense"

namespace brains2::control {

static casadi::Function generate_model(const Controller::ModelParams& params, size_t rk_steps = 1) {
    // State variables
    auto x = casadi::MX::sym("x", Controller::nx);
    auto split = vertsplit(x);
    auto s = split[0];
    auto n = split[1];
    auto psi = split[2];
    auto v = split[3];

    // control variables
    auto u = casadi::MX::sym("u", Controller::nu);
    split = vertsplit(u);
    auto delta = split[0];
    auto tau = split[1];

    // parameters
    auto kappa_cen = casadi::MX::sym("kappa_cen");

    // assemble the continuous dynamics
    IC();
    auto beta = params.l_R / (params.l_F + params.l_R) * delta;
    auto s_dot = v * cos(psi + beta) / (1 - n * kappa_cen);
    auto f_cont = casadi::Function(
        "f_cont",
        {x, u, kappa_cen},
        {cse(casadi::MX::vertcat({
            s_dot,
            v * sin(psi + beta),
            v * sin(beta) / params.l_R - kappa_cen * s_dot,
            (params.C_m0 * tau -
             (params.C_r0 + params.C_r1 * v + params.C_r2 * v * v) * tanh(10 * v)) /
                params.m,
        }))});

    IC();
    // Discretize with RK4
    auto xnext = x;
    auto scaled_dt = params.dt / rk_steps;
    for (size_t i = 0; i < rk_steps; ++i) {
        IC();
        auto k1 = f_cont({x, u, kappa_cen})[0];
        IC(x.size(), k1.size());
        auto k2 = f_cont({x + scaled_dt / 2 * k1, u, kappa_cen})[0];
        auto k3 = f_cont({x + scaled_dt / 2 * k2, u, kappa_cen})[0];
        auto k4 = f_cont({x + scaled_dt * k3, u, kappa_cen})[0];
        IC();
        xnext = x + scaled_dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    }
    IC();
    auto f_disc = casadi::Function("f_disc", {x, u, kappa_cen}, {cse(xnext)});
    return f_disc;
}

Controller::Controller(size_t Nf,
                       const Controller::ModelParams& model_params,
                       const Controller::Limits& limits,
                       const Controller::CostParams& cost_params,
                       size_t rk_steps)
    : Nf(Nf),
      dt(model_params.dt),
      v_ref(cost_params.v_ref),
      x_opt(Eigen::Matrix<double, State::dim, Eigen::Dynamic>::Zero(State::dim, Nf + 1)),
      u_opt(Eigen::Matrix<double, Control::dim, Eigen::Dynamic>::Zero(Control::dim, Nf)) {
    ///////////////////////////////////////////////////////////////////
    // Create optimization problem
    ///////////////////////////////////////////////////////////////////
    x.resize(Nf + 1);
    u.resize(Nf);
    // NOTE: the optimization variables have to be declared stage by stage
    // for fatrop to auto-detect the problem structure
    for (size_t i = 0; i < Nf; ++i) {
        x[i] = opti.variable(Controller::nx);
        u[i] = opti.variable(Controller::nu);
    }
    x[Nf] = opti.variable(Controller::nx);

    ///////////////////////////////////////////////////////////////////
    // Create optimization problem parameters
    ///////////////////////////////////////////////////////////////////
    x0 = opti.parameter(Controller::nx);  // initial state
    kappa_cen = opti.parameter(Nf);       // stages 0 to Nf-1
    w_cen = opti.parameter(Nf);           // stages 1 to Nf

    ///////////////////////////////////////////////////////////////////
    // Generate model
    ///////////////////////////////////////////////////////////////////
    auto f_disc = generate_model(model_params, rk_steps);

    ///////////////////////////////////////////////////////////////////
    // Construct cost function
    ///////////////////////////////////////////////////////////////////
    auto cost_function = casadi::MX(0.0);
    auto s0 = x0(0);
    auto Q = casadi::MX::diag(casadi::MX::vertcat(
             {cost_params.q_s, cost_params.q_n, cost_params.q_psi, cost_params.q_v})),
         R = casadi::MX::diag(casadi::MX::vertcat({cost_params.r_delta, cost_params.r_tau})),
         Q_f = casadi::MX::diag(casadi::MX::vertcat(
             {cost_params.q_s_f, cost_params.q_n_f, cost_params.q_psi_f, cost_params.q_v_f}));
    auto tau_ref = (model_params.C_r0 + model_params.C_r1 * cost_params.v_ref +
                    model_params.C_r2 * cost_params.v_ref * cost_params.v_ref) /
                   model_params.C_m0;
    for (size_t i = 0; i < Nf; ++i) {
        auto u_ref = casadi::DM({0.0, tau_ref});
        auto u_diff = u[i] - u_ref;
        cost_function += mtimes(u_diff.T(), mtimes(R, u_diff));
        if (i > 0) {
            auto x_ref =
                casadi::MX::vertcat({s0 + i / static_cast<double>(Nf) * cost_params.delta_s_ref,
                                     0.0,
                                     0.0,
                                     cost_params.v_ref});
            auto x_diff = x[i] - x_ref;
            cost_function += mtimes(x_diff.T(), mtimes(Q, x_diff));
        }
    }
    {
        auto x_ref =
            casadi::MX::vertcat({s0 + cost_params.delta_s_ref, 0.0, 0.0, cost_params.v_ref});
        auto x_diff = x[Nf] - x_ref;
        cost_function += mtimes(x_diff.T(), mtimes(Q, x_diff));
    }

    ///////////////////////////////////////////////////////////////////
    // Formulate constraints
    ///////////////////////////////////////////////////////////////////
    // NOTE: the constraints have to be declared stage by stage for
    // fatrop to properly auto-detect the problem structure.
    for (size_t i = 0; i < Nf; ++i) {
        if (i == 0) {
            opti.subject_to(x[0] == x0);
        }
        opti.subject_to(x[i + 1] == f_disc({x[i], u[i], kappa_cen(i)})[0]);
        opti.subject_to(opti.bounded(-limits.delta_max, u[i](0), limits.delta_max));
        opti.subject_to(opti.bounded(-limits.tau_max, u[i](1), limits.tau_max));
        if (i > 0) {
            opti.subject_to(opti.bounded(-w_cen(i - 1), x[i](1), w_cen(i - 1)));
            opti.subject_to(opti.bounded(0.0, x[i](3), limits.v_x_max));
        }
    }
    opti.subject_to(opti.bounded(-w_cen(Nf - 1), x[Nf](1), w_cen(Nf - 1)));
    opti.subject_to(opti.bounded(0.0, x[Nf](3), limits.v_x_max));

    ///////////////////////////////////////////////////////////////////
    // Solver and options
    ///////////////////////////////////////////////////////////////////
    opti.solver("fatrop",
                {
                    // {"print_time", 1},
                    {"expand", true},
                    {"debug", false},
                    {"structure_detection", "auto"},
                });
}

tl::expected<Controller::Control, Controller::ControllerError> Controller::compute_control(
    const Controller::State& current_state, const brains2::common::Track& track) {
    // Project current position
    auto [s, pos_proj] =
        track.project(current_state.X, current_state.Y, track.s_min(), track.length());

    // Convert to Frenet pose
    auto phi_proj = track.eval_phi(s);
    double n = -(current_state.X - pos_proj(0)) * sin(phi_proj) +
               (current_state.Y - pos_proj(1)) * cos(phi_proj),
           psi = current_state.phi - phi_proj;

    // Set current state
    opti.set_value(this->x0, casadi::DM({s, n, psi, current_state.v}));

    // Construct initial guess
    // Simplest way that does not depend on the previous call to the solver
    // is to create the initial guess based on the points at every dt*v_x_ref
    // TODO: optimize this by setting things in the constructor
    std::vector<double> kappa_cen_val(Nf), w_cen_val(Nf);
    for (size_t i = 0; i < Nf; ++i) {
        this->opti.set_initial(this->x[i], casadi::DM({s + i * dt * v_ref, 0.0, 0.0, v_ref}));
        this->opti.set_initial(this->u[i], casadi::DM({0.0, 0.0}));

        kappa_cen_val[i] = track.eval_kappa(s + i * dt * v_ref);
        if (i > 0) {
            w_cen_val[i] = track.eval_width(s + i * dt * v_ref);
        }
    }
    this->opti.set_initial(this->x[Nf], casadi::DM({s + Nf * dt * v_ref, 0.0, 0.0, v_ref}));
    w_cen_val[Nf - 1] = track.eval_width(s + Nf * dt * v_ref);
    this->opti.set_value(this->kappa_cen, kappa_cen_val);
    this->opti.set_value(this->w_cen, w_cen_val);

    // Call solver
    this->opti.solve_limited();

    // Check solver status
    const auto stats = this->opti.stats();
    if (!stats.at("success").as_bool()) {
        return tl::make_unexpected(Controller::ControllerError::UNKNOWN_ERROR);
    }
    // std::vector<casadi::DM> x_opt(Nf + 1), u_opt(Nf);
    casadi::DM tpr;
    for (size_t i = 0; i < Nf; ++i) {
        tpr = this->opti.value(this->x[i]);
        std::copy(tpr->data(), tpr->data() + tpr->size(), this->x_opt.data() + i * nx);
        tpr = this->opti.value(this->u[i]);
        std::copy(tpr->data(), tpr->data() + tpr->size(), this->u_opt.data() + i * nu);
    }
    tpr = this->opti.value(this->x[Nf]);
    std::copy(tpr->data(), tpr->data() + tpr->size(), this->x_opt.data() + Nf * nx);
    return Control{u_opt(0, 0), u_opt(1, 0)};
}

}  // namespace brains2::control
