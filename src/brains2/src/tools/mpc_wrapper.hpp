// Copyright 2025 Tudor Oancea, Mateo Berthet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef BRAINS2__TOOLS__MPC_WRAPPER_HPP_
#define BRAINS2__TOOLS__MPC_WRAPPER_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include "brains2/control/high_level_controller.hpp"
#include "brains2/common/track.hpp"

namespace brains2 {
namespace tools {

struct MPCParameters {
    // Model parameters (typically fixed)
    control::HighLevelController::ModelParams model_params;
    
    // Cost parameters (tunable)
    control::HighLevelController::CostParams cost_params;
    
    // Constraint parameters (tunable)
    control::HighLevelController::ConstraintsParams constraints_params;
    
    // Solver parameters
    control::HighLevelController::SolverParams solver_params;
    
    MPCParameters() {
        // Initialize model params
        model_params.dt = 0.05;
        model_params.m = 230.0;
        model_params.l_R = 0.7853;
        model_params.l_F = 0.7853;
        model_params.C_m0 = 4.950;
        model_params.C_r0 = 350.0;
        model_params.C_r1 = 20.0;
        model_params.C_r2 = 3.0;
        model_params.t_delta = 0.02;
        
        // Initialize cost params
        cost_params.v_ref = 3.0;
        cost_params.q_s = 10.0;
        cost_params.q_n = 20.0;
        cost_params.q_psi = 50.0;
        cost_params.q_v = 20.0;
        cost_params.r_delta = 2.0;
        cost_params.r_delta_dot = 1.0;
        cost_params.r_tau = 0.0001;
        cost_params.q_s_f = 10000.0;
        cost_params.q_n_f = 20000.0;
        cost_params.q_psi_f = 50000.0;
        cost_params.q_v_f = 20000.0;
        
        // Initialize constraint params
        constraints_params.v_max = 10.0;
        constraints_params.delta_max = 0.5;
        constraints_params.delta_dot_max = 1.0;
        constraints_params.tau_max = 200.0;
        constraints_params.car_width = 1.55;
        
        // Initialize solver params
        solver_params.jit = false;
        solver_params.solver = "fatrop";
    }
};

struct MPCResult {
    bool success;
    control::HighLevelController::Error error_code;
    double solve_time_ms;
    
    // State and control trajectories
    std::vector<control::HighLevelController::State> state_trajectory;
    std::vector<control::HighLevelController::Control> control_trajectory;
    
    // Time stamps for each step
    std::vector<double> time_stamps;
    
    // Cartesian trajectory for visualization
    std::vector<double> cart_X;
    std::vector<double> cart_Y;
};

class MPCWrapper {
public:
    static constexpr size_t HORIZON_NF = 20;
    
    MPCWrapper();
    explicit MPCWrapper(const MPCParameters& params);
    
    // Reconfigure MPC with new parameters
    void configure(const MPCParameters& params);
    
    // Check if MPC is properly configured
    bool is_configured() const;
    
    // Run open-loop MPC solve from initial state
    MPCResult solve_open_loop(
        const control::HighLevelController::State& initial_state,
        const common::Track& track
    );
    
    // Get timing statistics
    double last_solve_time_ms() const;
    double average_solve_time_ms() const;
    size_t total_solves() const;
    
    // Reset timing statistics
    void reset_timing_stats();
    
    // Get the current parameters
    const MPCParameters& get_parameters() const;
    
private:
    std::unique_ptr<control::HighLevelController> controller_;
    MPCParameters current_params_;
    bool configured_;
    
    // Timing statistics
    double last_solve_time_;
    double total_solve_time_;
    size_t num_solves_;
    
    // Convert Frenet state trajectory to Cartesian for visualization
    std::pair<std::vector<double>, std::vector<double>> frenet_to_cartesian_trajectory(
        const std::vector<control::HighLevelController::State>& states,
        const common::Track& track
    );
};

}  // namespace tools
}  // namespace brains2

#endif  // BRAINS2__TOOLS__MPC_WRAPPER_HPP_
