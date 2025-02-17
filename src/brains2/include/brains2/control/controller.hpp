// Copyright (c) 2024, Tudor Oancea, Matteo Berthet
#ifndef BRAINS2_CONTROL_CONTROLLER_HPP
#define BRAINS2_CONTROL_CONTROLLER_HPP

#include <array>
#include <cstdint>
#include <Eigen/Dense>
#include "brains2/common/tracks.hpp"
#include "brains2/external/expected.hpp"
#include "casadi/casadi.hpp"

namespace brains2 {
namespace control {

class Controller {
public:
    struct ModelParams {
        static constexpr uint8_t dim = 8;
        double dt, m, l_R, l_F, C_m0, C_r0, C_r1, C_r2;
    };
    struct CostParams {
        static constexpr uint8_t dim = 12;
        double v_ref, delta_s_ref, q_s, q_n, q_psi, q_v, r_delta, r_tau, q_s_f, q_n_f, q_psi_f,
            q_v_f;
    };
    struct Limits {
        static constexpr uint8_t dim = 3;
        double v_x_max, delta_max, tau_max;
    };
    struct State {
        static constexpr uint8_t dim = 4;
        double X, Y, phi, v;
    };
    struct Control {
        static constexpr uint8_t dim = 2;
        double delta, tau;
    };
    static constexpr uint8_t nx = State::dim;
    static constexpr uint8_t nu = Control::dim;

    Controller() = delete;
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;
    virtual ~Controller() = default;

    Controller(size_t Nf,
               const ModelParams& model_params,
               const Limits& limits,
               const CostParams& cost_params,
               size_t rk_steps = 1,
               bool jit = false);

    /*
     * @brief All the errors that may occur in the MPC.
     */
    enum class ControllerError : uint8_t {
        MAX_ITER,
        NANS_IN_SOLVER,
        INFEASIBLE_PROBLEM,
        UNKNOWN_ERROR,
    };

    static inline std::string to_string(ControllerError error) {
        switch (error) {
            case ControllerError::MAX_ITER:
                return "MAX_ITER";
            case ControllerError::NANS_IN_SOLVER:
                return "NANS_IN_SOLVER";
            case ControllerError::INFEASIBLE_PROBLEM:
                return "INFEASIBLE_PROBLEM";
            case ControllerError::UNKNOWN_ERROR:
                return "UNKNOWN_ERROR";
        }
    }

    /*
     * @brief Crunch the numbers; solve the MPC.
     * @param current_state The current state of the vehicle.
     * @param track The current track estimate.
     * @return A control command or an error.
     */
    tl::expected<Control, ControllerError> compute_control(const State& current_state,
                                                           const brains2::common::Track& track);

    /*
     * @brief Get the predicted positions of the vehicle in Cartesian coordinates.
     */
    Eigen::Matrix<double, 2, Eigen::Dynamic> get_predicted_positions() const;

    /*
     * @brief Const ref getter to the optimal state trajectory. May contain outdated data if
     * compute_control() returned an error.
     */
    inline const Eigen::Matrix<double, State::dim, Eigen::Dynamic>& get_x_opt() const {
        return x_opt;
    }

    /*
     * @brief Const ref getter to the optimal control trajectory. May contain outdated data if
     * compute_control() returned an error.
     */
    inline const Eigen::Matrix<double, Control::dim, Eigen::Dynamic>& get_u_opt() const {
        return u_opt;
    }

private:
    size_t Nf;
    double dt, v_ref, tau_ref;
    Eigen::Matrix<double, State::dim, Eigen::Dynamic> x_opt;
    Eigen::Matrix<double, Control::dim, Eigen::Dynamic> u_opt;
    casadi::Opti opti;
    casadi::MX cost_function;
    // optimization variables
    std::vector<casadi::MX> x, u;
    // params
    casadi::MX x0, kappa_cen, w_cen;
};

}  // namespace control
}  // namespace brains2

#endif
