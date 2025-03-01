// Copyright (c) 2024, Tudor Oancea, Matteo Berthet
#ifndef BRAINS2_CONTROL_CONTROLLER_HPP
#define BRAINS2_CONTROL_CONTROLLER_HPP

#include <array>
#include <cstdint>
#include <Eigen/Dense>
#include "brains2/common/tracks.hpp"
#include "brains2/external/expected.hpp"
#include "brains2/sim/sim.hpp"
#include "casadi/casadi.hpp"

namespace brains2 {
namespace control {

class Controller {
public:
    struct ModelParams {
        double dt, m, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_delta, C_downforce, z_CG, axle_track;
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

    Controller() = delete;
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;
    virtual ~Controller() = default;

    Controller(size_t Nf,
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
    tl::expected<Control, Error> compute_control(const State& current_state,
                                                 const brains2::common::Track& track);

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

std::string to_string(const Controller::Error& error);

/*
 * @brief Converts the control command internally used by the controller to the format used by the
 *        simulator. Concretely, it splits the global torque command into four torques (one for each
 *        wheel). For the moment it does so by evenly splitting them, without any torque vectoring.
 */
brains2::sim::Sim::Control to_sim_control(const Controller::Control& control);

brains2::sim::Sim::Control torque_vectoring_llc(const Controller::State& state,
                                                const double r,
                                                const double K_tv,
                                                const double a_x,
                                                const double a_y,
                                                const Controller::Control& control,
                                                const Controller::ModelParams& params);

}  // namespace control
}  // namespace brains2

#endif
