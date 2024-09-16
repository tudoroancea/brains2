// Copyright (c) 2024, Tudor Oancea, Matteo Berthet
#ifndef BRAINS2_SIM_SIM_HPP
#define BRAINS2_SIM_SIM_HPP

#include <casadi/mem.h>
#include <array>

namespace brains2 {
namespace sim {

class Sim {
public:
    struct State {
        static constexpr int dim = 11;
        double X, Y, phi, v_x, v_y, omega, delta, tau_FL, tau_FR, tau_RL, tau_RR;
    };
    struct Control {
        static constexpr int dim = 5;
        double u_delta, u_tau_FL, u_tau_FR, u_tau_RL, u_tau_RR;
    };
    struct Parameters {
        static constexpr int dim = 10;
        double m, I_z, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_T, t_delta;
    };
    struct Limits {
        double tau_max, delta_max, delta_dot_max;
    };
    struct Accels {
        static constexpr int dim = 2;
        double a_x, a_y;
    };
    static constexpr int nx = State::dim;
    static constexpr int nu = Control::dim;
    static constexpr int np = Parameters::dim;

private:
    // x = [X, Y, phi, v_x, v_y, omega, delta, tau_FL, tau_FR, tau_RL, tau_RR]
    std::array<double, nx> x;
    // u = [u_delta, u_tau_FL, u_tau_FR, u_tau_RL, u_tau_RR]
    std::array<double, nu> u;
    // p = [m, I_z, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_T, t_delta]
    std::array<double, np> p;
    // x_next = [X_next, Y_next, phi_next, v_x_next, v_y_next, omega_next, delta_next, tau_FL_next,
    // tau_FR_next, tau_RL_next, tau_RR_next]
    std::array<double, nx> x_next;
    // a = [a_x, a_y]
    std::array<double, 2> a;
    // Control limits used for the simulation
    Limits limits;
    // auxiliary memory for the CasADi function evaluation
    casadi_mem *mem;

public:
    Sim() = delete;
    Sim(const Parameters &params, const Limits &limits);
    virtual ~Sim();

    Sim(const Sim &other) = delete;
    Sim &operator=(const Sim &other) = delete;

    std::pair<State, Accels> simulate(const State &state, const Control &control, double dt);
};

}  // namespace sim
}  // namespace brains2

#endif