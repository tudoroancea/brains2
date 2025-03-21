// Copyright 2025 Tudor Oancea, Mateo Berthet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef BRAINS2__SIM__SIM_HPP_
#define BRAINS2__SIM__SIM_HPP_

#include <array>
#include <string>
#include <utility>
#include "acados/sim/sim_common.h"
#include "brains2/external/expected.hpp"
#include "casadi/mem.h"

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
        static constexpr int dim_kin6 = 10;
        static constexpr int dim_dyn6 = 17;
        double m, I_z, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_T, t_delta, z_CG, axle_track,
            C_downforce, Ba, Ca, Da, Ea;
    };
    struct Limits {
        double tau_max, delta_max, delta_dot_max;
    };
    struct Accels {
        double a_x, a_y;
    };

    /*
     * @brief Possible errors that can occur during a simulation step.
     */
    enum class Error {
        SAMPLING_TIME_UPDATE_ERROR = 0,
        ACADOS_SOLVER_ERROR = 1,
        ACCELS_FUNCTION_ERROR = 2,
        NANS_IN_RESULT = 3,
    };

    // We remove the default constructor, copy constructor and assignment operator because this
    // class has to allocate data on the heap (for the acados simulation solver and the casadi
    // function evaluation).
    Sim() = delete;
    Sim(const Sim &other) = delete;
    Sim &operator=(const Sim &other) = delete;

    /*
     * @brief Create a new simulation object.
     *
     * @param params The parameters of the model.
     * @param limits The control limits of the simulation.
     */
    Sim(const Parameters &params, const Limits &limits);
    virtual ~Sim();

    /*
     * @brief Run a simulation step, while enforcing the control limits and ensuring the car doesn't
     * driver in reverse.
     *
     * @param state The current state of the car.
     * @param control The target controls for the car, held constant during the simulation step.
     * @param dt The time to simlate for.
     * @return The next state of the car and the accelerations of the car if no erros occur,
     * otherwise a SimError.
     */
    tl::expected<std::pair<State, Accels>, Error> simulate(const State &state,
                                                           const Control &control,
                                                           double dt);

private:
    // Control limits used for the simulation
    Limits limits;
    // Arrays for the current state, target controls, next state and accelerations. These are used
    // for both the simulation solver and the CasADi function evaluation (for the accelerations).
    // x = [X, Y, phi, v_x, v_y, omega, delta, tau_FL, tau_FR, tau_RL, tau_RR]
    std::array<double, State::dim> x;
    // u = [u_delta, u_tau_FL, u_tau_FR, u_tau_RL, u_tau_RR]
    std::array<double, Control::dim> u;
    // p = [m, I_z, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_T, t_delta]
    std::array<double, Parameters::dim_dyn6> p;
    // x_next = [X_next, Y_next, phi_next, v_x_next, v_y_next, omega_next, delta_next, tau_FL_next,
    // tau_FR_next, tau_RL_next, tau_RR_next]
    std::array<double, State::dim> x_next;
    // a = [a_x, a_y]
    std::array<double, 2> a;

    struct sim_workspace {
        // Auxiliary memory for the CasADi function evaluation
        casadi_mem *accel_fun_mem;
        // Acados simulation solver variables
        void *_sim_capsule;
        sim_config *_sim_config;
        sim_in *_sim_in;
        sim_out *_sim_out;
        void *_sim_dims;
        // Auxiliary functions
        // int (*update_params)(void *, const double *, int);
        // int (*solve)(void *);
        // int
    } kin6_workspace, dyn6_workspace;
};

std::string to_string(Sim::Error error);

}  // namespace sim
}  // namespace brains2

#endif  // BRAINS2__SIM__SIM_HPP_
