// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/control/high_level_controller.hpp"
#include <cmath>
#include <iterator>
#include "brains2/common/tracks.hpp"
#include "brains2/external/expected.hpp"
#include "brains2/external/icecream.hpp"
#include "casadi/casadi.hpp"
#include "Eigen/Dense"

namespace brains2::control {

static casadi::Function generate_model(const HighLevelController::ModelParams& params,
                                       size_t rk_steps = 1) {
    // State variables
    auto x = casadi::MX::sym("x", HighLevelController::nx);
    auto split = vertsplit(x);
    auto Delta_s = split[0];
    auto n = split[1];
    auto psi = split[2];
    auto v = split[3];
    auto delta = split[4];

    // control variables
    auto u = casadi::MX::sym("u", HighLevelController::nu);
    split = vertsplit(u);
    auto u_delta = split[0];
    auto tau = split[1];

    // parameters
    auto kappa_cen = casadi::MX::sym("kappa_cen");

    // assemble the continuous dynamics
    auto beta = params.l_R / (params.l_F + params.l_R) * u_delta;
    auto s_dot = v * cos(psi + beta) / (1 + n * kappa_cen);
    auto f_cont = casadi::Function(
        "f_cont",
        {x, u, kappa_cen},
        {casadi::MX::vertcat({
            s_dot,
            v * sin(psi + beta),
            v * sin(beta) / params.l_R + kappa_cen * s_dot,
            (params.C_m0 * tau - (params.C_r0 + params.C_r1 * v + params.C_r2 * v * v)) / params.m,
            (u_delta - delta) / params.t_delta,
        })});

    // Discretize with RK4
    auto xnext = x;
    auto scaled_dt = params.dt / rk_steps;
    for (size_t i = 0; i < rk_steps; ++i) {
        auto k1 = f_cont({xnext, u, kappa_cen})[0];
        auto k2 = f_cont({xnext + scaled_dt / 2 * k1, u, kappa_cen})[0];
        auto k3 = f_cont({xnext + scaled_dt / 2 * k2, u, kappa_cen})[0];
        auto k4 = f_cont({xnext + scaled_dt * k3, u, kappa_cen})[0];
        xnext += scaled_dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    }
    auto f_disc = casadi::Function("f_disc", {x, u, kappa_cen}, {cse(xnext)});
    return f_disc;
}

HighLevelController::HighLevelController(
    size_t Nf,
    const HighLevelController::ModelParams& model_params,
    const HighLevelController::ConstraintsParams& constraints_params,
    const HighLevelController::CostParams& cost_params,
    const HighLevelController::SolverParams& solver_params)
    : Nf(Nf),
      dt(model_params.dt),
      v_ref(cost_params.v_ref),
      tau_ref((model_params.C_r0 + model_params.C_r1 * cost_params.v_ref +
               model_params.C_r2 * cost_params.v_ref * cost_params.v_ref) /
              model_params.C_m0),
      x_ref(HighLevelController::StateHorizonMatrix::Zero(nx, Nf + 1)),
      u_ref(HighLevelController::ControlHorizonMatrix::Zero(nu, Nf)),
      x_opt(HighLevelController::StateHorizonMatrix::Zero(nx, Nf + 1)),
      u_opt(HighLevelController::ControlHorizonMatrix::Zero(nu, Nf)) {
    ///////////////////////////////////////////////////////////////////
    // Create optimization problem
    ///////////////////////////////////////////////////////////////////
    x.resize(Nf + 1);
    u.resize(Nf);
    // NOTE: the optimization variables have to be declared stage by stage
    // for fatrop to auto-detect the problem structure
    for (size_t i = 0; i < Nf; ++i) {
        x[i] = opti.variable(HighLevelController::nx);
        u[i] = opti.variable(HighLevelController::nu);
    }
    x[Nf] = opti.variable(HighLevelController::nx);

    ///////////////////////////////////////////////////////////////////
    // Create optimization problem parameters
    ///////////////////////////////////////////////////////////////////
    x0 = opti.parameter(HighLevelController::nx);
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
    const auto Q = casadi::MX::diag(casadi::MX::vertcat({cost_params.q_s,
                                                         cost_params.q_n,
                                                         cost_params.q_psi,
                                                         cost_params.q_v,
                                                         cost_params.r_delta})),
               R = casadi::MX::diag(casadi::MX::vertcat({cost_params.r_delta, cost_params.r_tau})),
               Q_f = casadi::MX::diag(casadi::MX::vertcat({cost_params.q_s_f,
                                                           cost_params.q_n_f,
                                                           cost_params.q_psi_f,
                                                           cost_params.q_v_f,
                                                           cost_params.r_delta}));
    cost_function = casadi::MX(0.0);
    for (size_t i = 0; i <= Nf; ++i) {
        if (i < Nf) {
            const auto u_diff = u[i] - casadi::DM({u_ref(0, i), u_ref(1, i)});
            cost_function += mtimes(u_diff.T(), mtimes(R, u_diff));
            // cost_function += cost_params.r_delta_dot * (u[i](0) - x[i](4)) * (u[i](0) - x[i](4));
        }
        if (i > 0) {
            const auto x_diff =
                x[i] -
                casadi::DM({x_ref(0, i), x_ref(1, i), x_ref(2, i), x_ref(3, i), x_ref(4, i)});
            cost_function += mtimes(x_diff.T(), mtimes(Q, x_diff));
        }
    }
    opti.minimize(cse(cost_function));

    ///////////////////////////////////////////////////////////////////
    // Formulate constraints
    ///////////////////////////////////////////////////////////////////
    // NOTE: the constraints have to be declared stage by stage for
    // fatrop to properly auto-detect the problem structure.
    for (size_t i = 0; i <= Nf; ++i) {
        // Dynamics
        if (i < Nf) {
            opti.subject_to(x[i + 1] == f_disc({x[i], u[i], kappa_cen(i)})[0]);
        }
        // Initial conditions
        if (i == 0) {
            opti.subject_to(x[0] == x0);
        }
        // Control constraints
        if (i < Nf) {
            opti.subject_to(
                opti.bounded(-constraints_params.delta_max, u[i](0), constraints_params.delta_max));
            opti.subject_to(
                opti.bounded(-constraints_params.tau_max, u[i](1), constraints_params.tau_max));
            opti.subject_to(opti.bounded(-constraints_params.delta_dot_max * model_params.t_delta,
                                         u[i](0) - x[i](4),
                                         constraints_params.delta_dot_max * model_params.t_delta));
        }
        // State constraints
        if (i > 0) {
            opti.subject_to(opti.bounded(-w_cen(i) + constraints_params.car_width / 2,
                                         x[i](1),
                                         w_cen(i) - constraints_params.car_width / 2));
            opti.subject_to(opti.bounded(0.0, x[i](3), constraints_params.v_max));
        }
    }

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
    for (size_t i = 0; i <= Nf; ++i) {
        if (i < Nf) {
            this->opti.set_initial(this->u[i], casadi::DM({u_ref(0, i), u_ref(1, i)}));
        }
        this->opti.set_initial(
            this->x[i],
            casadi::DM({x_ref(0, i), x_ref(1, i), x_ref(2, i), x_ref(3, i), x_ref(4, i)}));
    }
}

tl::expected<std::vector<HighLevelController::Control>, HighLevelController::Error>
HighLevelController::compute_control(const HighLevelController::State& state,
                                     const brains2::common::Track& track) {
    // Set current state
    this->opti.set_value(this->x0, casadi::DM({0.0, state.n, state.psi, state.v, state.delta}));

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

        // Extract solution
        casadi::DM tpr;
        std::vector<Control> controls(Nf);
        for (size_t i = 0; i <= Nf; ++i) {
            if (i < Nf) {
                tpr = sol.value(this->u[i]);
                std::copy(tpr->begin(), tpr->end(), this->u_opt.data() + i * nu);
                controls[i] = Control{u_opt(0, i), u_opt(1, i)};
            }
            tpr = sol.value(this->x[i]);
            std::copy(tpr->begin(), tpr->end(), this->x_opt.data() + i * nx);
        }

        return controls;
    } catch (const std::exception& e) {
        IC(e.what());
        // TODO: handle all types of return status
        const auto stats = this->opti.stats();
        IC(stats.at("unified_return_status"), opti.return_status());
        return tl::make_unexpected(HighLevelController::Error::UNKNOWN_ERROR);
    }
}

std::string to_string(const HighLevelController::Error& error) {
    switch (error) {
        case HighLevelController::Error::MAX_ITER:
            return "MAX_ITER";
        case HighLevelController::Error::NANS_IN_SOLVER:
            return "NANS_IN_SOLVER";
        case HighLevelController::Error::INFEASIBLE_PROBLEM:
            return "INFEASIBLE_PROBLEM";
        case HighLevelController::Error::UNKNOWN_ERROR:
            return "UNKNOWN_ERROR";
    }
}

}  // namespace brains2::control
