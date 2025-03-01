// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/control/controller.hpp"
#include <cmath>
#include <iterator>
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
    auto Delta_s = split[0];
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
    auto beta = params.l_R / (params.l_F + params.l_R) * delta;
    auto s_dot = v * cos(psi + beta) / (1 + n * kappa_cen);
    auto f_cont = casadi::Function(
        "f_cont",
        {x, u, kappa_cen},
        {casadi::MX::vertcat({
            s_dot,
            v * sin(psi + beta),
            v * sin(beta) / params.l_R + kappa_cen * s_dot,
            (params.C_m0 * tau - (params.C_r0 + params.C_r1 * v + params.C_r2 * v * v)) / params.m,
        })});

    // Discretize with RK4
    auto xnext = x;
    auto scaled_dt = params.dt / rk_steps;
    for (size_t i = 0; i < rk_steps; ++i) {
        auto k1 = f_cont({x, u, kappa_cen})[0];
        auto k2 = f_cont({x + scaled_dt / 2 * k1, u, kappa_cen})[0];
        auto k3 = f_cont({x + scaled_dt / 2 * k2, u, kappa_cen})[0];
        auto k4 = f_cont({x + scaled_dt * k3, u, kappa_cen})[0];
        xnext = x + scaled_dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    }
    auto f_disc = casadi::Function("f_disc", {x, u, kappa_cen}, {cse(xnext)});
    return f_disc;
}

Controller::Controller(size_t Nf,
                       const Controller::ModelParams& model_params,
                       const Controller::ConstraintsParams& contraints_params,
                       const Controller::CostParams& cost_params,
                       const Controller::SolverParams& solver_params)
    : Nf(Nf),
      dt(model_params.dt),
      v_ref(cost_params.v_ref),
      tau_ref((model_params.C_r0 + model_params.C_r1 * cost_params.v_ref +
               model_params.C_r2 * cost_params.v_ref * cost_params.v_ref) /
              model_params.C_m0),
      x_ref(Controller::StateHorizonMatrix::Zero(nx, Nf + 1)),
      u_ref(Controller::ControlHorizonMatrix::Zero(nu, Nf)),
      x_opt(Controller::StateHorizonMatrix::Zero(nx, Nf + 1)),
      u_opt(Controller::ControlHorizonMatrix::Zero(nu, Nf)) {
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
    x0 = opti.parameter(Controller::nx);
    kappa_cen = opti.parameter(Nf + 1);
    w_cen = opti.parameter(Nf + 1);

    ///////////////////////////////////////////////////////////////////
    // Generate model
    ///////////////////////////////////////////////////////////////////
    auto f_disc = generate_model(model_params);

    ///////////////////////////////////////////////////////////////////
    // Construct cost function
    ///////////////////////////////////////////////////////////////////
    // Create reference
    u_ref.row(1).array() = tau_ref;
    x_ref.row(0) = Eigen::VectorXd::LinSpaced(Nf + 1, 0.0, Nf * dt * v_ref);
    x_ref.row(3).array() = v_ref;
    // Create cost weight matrices
    const auto Q = casadi::MX::diag(casadi::MX::vertcat(
                   {cost_params.q_s, cost_params.q_n, cost_params.q_psi, cost_params.q_v})),
               R = casadi::MX::diag(casadi::MX::vertcat({cost_params.r_delta, cost_params.r_tau})),
               Q_f = casadi::MX::diag(casadi::MX::vertcat(
                   {cost_params.q_s_f, cost_params.q_n_f, cost_params.q_psi_f, cost_params.q_v_f}));
    cost_function = casadi::MX(0.0);
    for (size_t i = 0; i < Nf; ++i) {
        const auto u_diff = u[i] - casadi::DM({u_ref(0, i), u_ref(1, i)});
        cost_function += mtimes(u_diff.T(), mtimes(R, u_diff));
        if (i > 0) {
            const auto x_diff =
                x[i] - casadi::DM({x_ref(0, i), x_ref(1, i), x_ref(2, i), x_ref(3, i)});
            cost_function += mtimes(x_diff.T(), mtimes(Q, x_diff));
        }
    }
    {
        auto x_diff = x[Nf] - casadi::DM({x_ref(0, Nf), x_ref(1, Nf), x_ref(2, Nf), x_ref(3, Nf)});
        cost_function += mtimes(x_diff.T(), mtimes(Q, x_diff));
    }
    opti.minimize(cse(cost_function));

    ///////////////////////////////////////////////////////////////////
    // Formulate constraints
    ///////////////////////////////////////////////////////////////////
    // NOTE: the constraints have to be declared stage by stage for
    // fatrop to properly auto-detect the problem structure.
    for (size_t i = 0; i < Nf; ++i) {
        opti.subject_to(x[i + 1] == f_disc({x[i], u[i], kappa_cen(i)})[0]);
        if (i == 0) {
            opti.subject_to(x[0] == x0);
        }
        opti.subject_to(
            opti.bounded(-contraints_params.delta_max, u[i](0), contraints_params.delta_max));
        opti.subject_to(
            opti.bounded(-contraints_params.tau_max, u[i](1), contraints_params.tau_max));
        if (i > 0) {
            opti.subject_to(opti.bounded(-w_cen(i) + contraints_params.car_width / 2,
                                         x[i](1),
                                         w_cen(i) - contraints_params.car_width / 2));
            opti.subject_to(opti.bounded(0.0, x[i](3), contraints_params.v_x_max));
        }
    }
    opti.subject_to(opti.bounded(-w_cen(Nf) - contraints_params.car_width / 2,
                                 x[Nf](1),
                                 w_cen(Nf) - contraints_params.car_width / 2));
    opti.subject_to(opti.bounded(0.0, x[Nf](3), contraints_params.v_x_max));

    ///////////////////////////////////////////////////////////////////
    // Solver and options
    ///////////////////////////////////////////////////////////////////
    if (solver_params.solver == "fatrop") {
        opti.solver(
            "fatrop",
            {
                {"print_time", 0},
                {"expand", true},
                {"debug", false},
                {"structure_detection", "auto"},
                {"fatrop", casadi::Dict({{"print_level", 1}})},
                {"jit", solver_params.jit},
                {"jit_options", casadi::Dict({{"flags", "-O2 -march=native"}, {"verbose", false}})},
            });
    } else if (solver_params.solver == "ipopt") {
        opti.solver(
            "ipopt",
            {
                {"print_time", 0},
                {"expand", true},
                {"ipopt", casadi::Dict({{"print_level", 0}})},
                {"jit", solver_params.jit},
                {"jit_options", casadi::Dict({{"flags", "-O2 -march=native"}, {"verbose", false}})},
            });
    } else {
        throw std::runtime_error("Unknown solver");
    }

    ///////////////////////////////////////////////////////////////////
    // Set initial guess
    ///////////////////////////////////////////////////////////////////
    for (size_t i = 0; i < Nf; ++i) {
        this->opti.set_initial(this->x[i],
                               casadi::DM({x_ref(0, i), x_ref(1, i), x_ref(2, i), x_ref(3, i)}));
        this->opti.set_initial(this->u[i], casadi::DM({u_ref(0, i), u_ref(1, i)}));
    }
    this->opti.set_initial(this->x[Nf],
                           casadi::DM({x_ref(0, Nf), x_ref(1, Nf), x_ref(2, Nf), x_ref(3, Nf)}));
}

tl::expected<Controller::Control, Controller::Error> Controller::compute_control(
    const Controller::State& state, const brains2::common::Track& track) {
    // Set current state
    this->opti.set_value(this->x0, casadi::DM({0.0, state.n, state.psi, state.v}));

    // Compute parameter values
    std::vector<double> kappa_cen_val(Nf + 1), w_cen_val(Nf + 1);
    for (size_t i = 0; i < Nf + 1; ++i) {
        kappa_cen_val[i] = track.eval_kappa(state.s + x_ref(0, i));
        w_cen_val[i] = track.eval_width(state.s + x_ref(0, i));
    }
    this->opti.set_value(this->kappa_cen, kappa_cen_val);
    this->opti.set_value(this->w_cen, w_cen_val);

    try {
        // Call solver
        auto sol = this->opti.solve();

        // Check solver status
        const auto stats = this->opti.stats();
        if (!stats.at("success").as_bool()) {
            throw std::runtime_error("Solver failed");
        }

        // Extract solution
        casadi::DM tpr;
        for (size_t i = 0; i < Nf; ++i) {
            tpr = sol.value(this->x[i]);
            std::copy(tpr->begin(), tpr->end(), this->x_opt.data() + i * nx);
            tpr = sol.value(this->u[i]);
            std::copy(tpr->begin(), tpr->end(), this->u_opt.data() + i * nu);
        }
        tpr = sol.value(this->x[Nf]);
        std::copy(tpr->begin(), tpr->end(), this->x_opt.data() + Nf * nx);
        return Control{u_opt(0, 0), u_opt(1, 0)};
    } catch (const std::exception& e) {
        IC(e.what());
        // TODO: handle all types of return status
        const auto stats = this->opti.stats();
        IC(stats.at("unified_return_status"), opti.return_status());
        return tl::make_unexpected(Controller::Error::UNKNOWN_ERROR);
    }
}

std::string to_string(const Controller::Error& error) {
    switch (error) {
        case Controller::Error::MAX_ITER:
            return "MAX_ITER";
        case Controller::Error::NANS_IN_SOLVER:
            return "NANS_IN_SOLVER";
        case Controller::Error::INFEASIBLE_PROBLEM:
            return "INFEASIBLE_PROBLEM";
        case Controller::Error::UNKNOWN_ERROR:
            return "UNKNOWN_ERROR";
    }
}

brains2::sim::Sim::Control to_sim_control(const Controller::Control& control) {
    return brains2::sim::Sim::Control{control.delta,
                                      control.tau / 4,
                                      control.tau / 4,
                                      control.tau / 4,
                                      control.tau / 4};
}

}  // namespace brains2::control
