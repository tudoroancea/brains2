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
// FITNESS FOR A PARTICULAR PURPOSE AND ININFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mpc_wrapper.hpp"
#include <iostream>

namespace brains2 {
namespace tools {

MPCWrapper::MPCWrapper()
    : controller_(nullptr),
      configured_(false),
      last_solve_time_(0.0),
      total_solve_time_(0.0),
      num_solves_(0) {
}

MPCWrapper::MPCWrapper(const MPCParameters& params) : MPCWrapper() {
    configure(params);
}

void MPCWrapper::configure(const MPCParameters& params) {
    current_params_ = params;
    
    try {
        controller_ = std::make_unique<control::HighLevelController>(
            HORIZON_NF,
            params.model_params,
            params.constraints_params,
            params.cost_params,
            params.solver_params
        );
        configured_ = true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to create MPC controller: " << e.what() << std::endl;
        configured_ = false;
    }
}

bool MPCWrapper::is_configured() const {
    return configured_ && controller_ != nullptr;
}

MPCResult MPCWrapper::solve_open_loop(
    const control::HighLevelController::State& initial_state,
    const common::Track& track
) {
    MPCResult result;
    result.success = false;
    result.error_code = control::HighLevelController::Error::UNKNOWN_ERROR;
    result.solve_time_ms = 0.0;
    result.state_trajectory.clear();
    result.control_trajectory.clear();
    result.time_stamps.clear();
    result.cart_X.clear();
    result.cart_Y.clear();
    
    if (!is_configured()) {
        std::cerr << "MPC not configured" << std::endl;
        result.error_message = "MPC not configured. Load a track first.";
        return result;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Solve MPC
    auto control_result = controller_->compute_control(initial_state, track);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.solve_time_ms = duration.count() / 1000.0;
    
    // Update timing statistics
    last_solve_time_ = result.solve_time_ms;
    total_solve_time_ += result.solve_time_ms;
    num_solves_++;
    
    if (!control_result.has_value()) {
        result.error_code = control_result.error();
        result.error_message = "MPC solve failed: " + brains2::control::to_string(result.error_code);
        std::cerr << result.error_message << std::endl;
        return result;
    }
    
    result.success = true;
    
    // Extract state and control trajectories
    const auto& x_opt = controller_->get_x_opt();
    const auto& u_opt = controller_->get_u_opt();
    double dt = current_params_.model_params.dt;
    
    result.state_trajectory.resize(HORIZON_NF + 1);
    result.control_trajectory.resize(HORIZON_NF);
    result.time_stamps.resize(HORIZON_NF + 1);
    
    // First state is the initial state
    result.state_trajectory[0] = initial_state;
    result.time_stamps[0] = 0.0;
    
    // Extract optimal states from x_opt (columns are time steps)
    for (size_t i = 0; i <= HORIZON_NF; i++) {
        if (i > 0) {
            result.state_trajectory[i] = {
                x_opt(0, i),  // s
                x_opt(1, i),  // n
                x_opt(2, i),  // psi
                x_opt(3, i),  // v
                x_opt(4, i)   // delta
            };
            result.time_stamps[i] = i * dt;
        }
    }
    
    // Extract optimal controls from u_opt (columns are time steps)
    for (size_t i = 0; i < HORIZON_NF; i++) {
        result.control_trajectory[i] = {
            u_opt(0, i),  // u_delta
            u_opt(1, i)   // tau
        };
    }
    
    // Convert to Cartesian for visualization
    auto [cart_X, cart_Y] = frenet_to_cartesian_trajectory(result.state_trajectory, track);
    result.cart_X = std::move(cart_X);
    result.cart_Y = std::move(cart_Y);
    
    return result;
}

double MPCWrapper::last_solve_time_ms() const {
    return last_solve_time_;
}

double MPCWrapper::average_solve_time_ms() const {
    if (num_solves_ == 0) return 0.0;
    return total_solve_time_ / static_cast<double>(num_solves_);
}

size_t MPCWrapper::total_solves() const {
    return num_solves_;
}

void MPCWrapper::reset_timing_stats() {
    last_solve_time_ = 0.0;
    total_solve_time_ = 0.0;
    num_solves_ = 0;
}

const MPCParameters& MPCWrapper::get_parameters() const {
    return current_params_;
}

std::pair<std::vector<double>, std::vector<double>> MPCWrapper::frenet_to_cartesian_trajectory(
        const std::vector<control::HighLevelController::State>& states,
        const common::Track& track) {
    std::vector<double> cart_X;
    std::vector<double> cart_Y;
    
    cart_X.reserve(states.size());
    cart_Y.reserve(states.size());
    
    for (const auto& state : states) {
        common::FrenetPose frenet_pose = {state.s, state.n, state.psi};
        auto result = track.frenet_to_cartesian(frenet_pose);
        
        // First CartesianPose is the actual position, second is projection
        cart_X.push_back(result.first.X);
        cart_Y.push_back(result.first.Y);
    }
    
    return {std::move(cart_X), std::move(cart_Y)};
}

}  // namespace tools
}  // namespace brains2
