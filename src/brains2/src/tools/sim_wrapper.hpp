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

#ifndef BRAINS2__TOOLS__SIM_WRAPPER_HPP_
#define BRAINS2__TOOLS__SIM_WRAPPER_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include "brains2/control/high_level_controller.hpp"
#include "brains2/sim/sim.hpp"
#include "brains2/common/track.hpp"
#include "mpc_wrapper.hpp"

namespace brains2 {
namespace tools {

struct SimParameters {
    sim::Sim::Parameters sim_params;
    sim::Sim::Limits sim_limits;
    
    SimParameters() {
        // Initialize sim params
        sim_params.m = 230.0;
        sim_params.I_z = 400.0;
        sim_params.l_R = 0.7853;
        sim_params.l_F = 0.7853;
        sim_params.C_m0 = 4.950;
        sim_params.C_r0 = 350.0;
        sim_params.C_r1 = 20.0;
        sim_params.C_r2 = 3.0;
        sim_params.t_T = 0.0;
        sim_params.t_delta = 0.02;
        sim_params.z_CG = 0.3;
        sim_params.axle_track = 1.55;
        sim_params.C_downforce = 0.0;
        sim_params.Ba = 0.0;
        sim_params.Ca = 0.0;
        sim_params.Da = 0.0;
        sim_params.Ea = 0.0;
        
        // Initialize sim limits
        sim_limits.tau_max = 200.0;
        sim_limits.delta_max = 0.5;
        sim_limits.delta_dot_max = 1.0;
    }
};

struct OpenLoopSimResult {
    bool success;
    std::string error_message;
    double total_time_ms;
    double sim_step_time_ms;
    
    // Full simulation trajectory
    std::vector<control::HighLevelController::State> frenet_states;
    std::vector<control::HighLevelController::Control> controls;
    std::vector<double> time_stamps;
    
    // Cartesian trajectory for visualization
    std::vector<double> cart_X;
    std::vector<double> cart_Y;
    
    // Number of simulation steps
    size_t num_steps;
};

class SimWrapper {
public:
    SimWrapper();
    explicit SimWrapper(const SimParameters& params);
    
    void configure(const SimParameters& params);
    bool is_configured() const;
    
    // Run open-loop simulation: apply the HLC's predicted control sequence to the full dynamics model
    // (no feedback, no re-solving MPC at each step)
    OpenLoopSimResult run_open_loop_simulation(
        control::HighLevelController::State initial_state,
        const common::Track& track,
        const MPCParameters& mpc_params,
        const std::vector<control::HighLevelController::Control>& control_sequence
    );
    
    // Get timing statistics
    double last_total_time_ms() const;
    double average_sim_step_time_ms() const;
    size_t total_runs() const;
    
    void reset_timing_stats();
    
private:
    std::unique_ptr<sim::Sim> sim_;
    SimParameters current_params_;
    bool configured_;
    
    // Timing statistics
    double last_total_time_;
    double total_sim_time_;
    size_t num_runs_;
    
    // Convert Frenet state to Sim state
    sim::Sim::State frenet_to_sim_state(
        const control::HighLevelController::State& frenet_state,
        const common::Track& track
    );
    
    // Convert Sim state back to Frenet state
    control::HighLevelController::State sim_to_frenet_state(
        const sim::Sim::State& sim_state,
        const common::Track& track
    );
    
    // Convert Frenet state trajectory to Cartesian for visualization
    std::pair<std::vector<double>, std::vector<double>> frenet_to_cartesian_trajectory(
        const std::vector<control::HighLevelController::State>& states,
        const common::Track& track
    );
};

}  // namespace tools
}  // namespace brains2

#endif  // BRAINS2__TOOLS__SIM_WRAPPER_HPP_
