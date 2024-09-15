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
    static constexpr int nx = State::dim;
    static constexpr int nu = Control::dim;
    static constexpr int np = Parameters::dim;

private:
    std::array<double, nx> x;
    std::array<double, nx> x_next;
    std::array<double, nu> u;
    std::array<double, 2> accels;
    std::array<double, np> p;
    casadi_mem *mem;

public:
    Sim();
    ~Sim();

    Sim(const Sim &other) = delete;
    Sim &operator=(const Sim &other) = delete;

    void reset();

    void step(const Sim::Control &u, double dt);
    Sim::State get_current_state();
    std::pair<double, double> get_current_accels();
};

}  // namespace sim
}  // namespace brains2

#endif
