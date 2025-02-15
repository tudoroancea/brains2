// Copyright (c) 2024, Tudor Oancea, Matteo Berthet
#ifndef BRAINS2_CONTROL_CONTROLLER_HPP
#define BRAINS2_CONTROL_CONTROLLER_HPP

#include <array>
#include <cstdint>
#include "brains2/common/tracks.hpp"
#include "brains2/external/expected.hpp"
#include "casadi/mem.h"

namespace brains2 {
namespace control {

class Controller {
public:
    static constexpr uint8_t Nf = 10;

    struct ModelParams {
        static constexpr uint8_t dim = 10;
        double dt, m, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_delta, t_tau;
    };
    struct CostParams {
        static constexpr uint8_t dim = 20;
        double v_x_ref, delta_s_ref, q_s, q_n, q_psi, q_v_x, q_v_y, q_omega, q_delta, q_tau,
            r_delta, r_tau, q_s_f, q_n_f, q_psi_f, q_v_x_f, q_v_y_f, q_omega_f, q_delta_f, q_tau_f;
    };
    struct Limits {
        static constexpr uint8_t dim = 3;
        double v_x_max, delta_max, tau_max;
    };

    struct State {
        static constexpr uint8_t dim = 8;
        double X, Y, phi, v_x, v_y, omega, delta, tau;
    };
    struct Control {
        static constexpr uint8_t dim = 2;
        double u_delta, u_tau;
    };

    Controller() = delete;
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;
    virtual ~Controller() = default;

    Controller(const ModelParams& model_params,
               const Limits& limits,
               const CostParams& cost_params);

    enum class ControllerError : uint8_t {
        MAX_ITER = 0,
    };
    static inline std::string to_string(ControllerError error) {
        switch (error) {
            case ControllerError::MAX_ITER:
                return "MAX_ITER";
            default:
                return "UNKNOWN_ERROR";
        }
    }

    tl::expected<Control, ControllerError> compute_control(const State& current_state,
                                                           const brains2::common::Track& track);

private:
    // Casadi solver function memory
    casadi_mem* solver_fun_mem;
    // Function inputs
    std::array<double, State::dim*(Nf + 1)> x_guess;
    std::array<double, Control::dim * Nf> u_guess;
    std::array<double, State::dim> x0;
    std::array<double, Nf> kappa_cen;
    std::array<double, Nf> w_cen;
    std::array<double, ModelParams::dim> model_params;
    std::array<double, Limits::dim> limits;
    std::array<double, CostParams::dim> cost_params;
    // Function outputs
    std::array<double, State::dim*(Nf + 1)> x_opt;
    std::array<double, Control::dim * Nf> u_opt;
    // Redundant variables that are simpler to access
    double dt, v_x_ref;
};

}  // namespace control
}  // namespace brains2

#endif
