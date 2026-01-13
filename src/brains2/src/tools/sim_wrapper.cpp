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

#include "sim_wrapper.hpp"
#include "mpc_wrapper.hpp"
#include <iostream>
#include <cmath>

namespace brains2 {
namespace tools {

SimWrapper::SimWrapper()
    : sim_(nullptr),
      configured_(false),
      last_total_time_(0.0),
      total_sim_time_(0.0),
      num_runs_(0) {
}

SimWrapper::SimWrapper(const SimParameters& params) : SimWrapper() {
    configure(params);
}

void SimWrapper::configure(const SimParameters& params) {
    current_params_ = params;
    
    try {
        sim_ = std::make_unique<sim::Sim>(params.sim_params, params.sim_limits);
        configured_ = true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to create Sim: " << e.what() << std::endl;
        configured_ = false;
    }
}

bool SimWrapper::is_configured() const {
    return configured_ && sim_ != nullptr;
}

OpenLoopSimResult SimWrapper::run_open_loop_simulation(
    control::HighLevelController::State initial_state,
    const common::Track& track,
    const MPCParameters& mpc_params,
    const std::vector<control::HighLevelController::Control>& control_sequence
) {
    OpenLoopSimResult result;
    result.success = false;
    result.error_message = "";
    result.total_time_ms = 0.0;
    result.sim_step_time_ms = 0.0;
    result.num_steps = 0;
    
    if (!is_configured()) {
        result.error_message = "Sim not configured";
        return result;
    }
    
    auto start_total = std::chrono::high_resolution_clock::now();
    
    double dt = mpc_params.model_params.dt;
    size_t num_steps = control_sequence.size();
    
    // Initialize state
    control::HighLevelController::State current_state = initial_state;
    
    // Store trajectory
    result.frenet_states.reserve(num_steps + 1);
    result.controls = control_sequence;  // Store the control sequence
    result.time_stamps.reserve(num_steps + 1);
    
    result.frenet_states.push_back(current_state);
    result.time_stamps.push_back(0.0);
    
    for (size_t step = 0; step < num_steps; step++) {
        // Get control from sequence
        const control::HighLevelController::Control& ctrl = control_sequence[step];
        
        // Convert to Sim state and step
        sim::Sim::State sim_state = frenet_to_sim_state(current_state, track);
        
        // Create Sim control (steering rate and torque)
        sim::Sim::Control sim_ctrl;
        sim_ctrl.u_delta = ctrl.u_delta;  // steering rate
        sim_ctrl.u_tau_FL = ctrl.tau / 4.0;
        sim_ctrl.u_tau_FR = ctrl.tau / 4.0;
        sim_ctrl.u_tau_RL = ctrl.tau / 4.0;
        sim_ctrl.u_tau_RR = ctrl.tau / 4.0;
        
        auto sim_start = std::chrono::high_resolution_clock::now();
        auto sim_result = sim_->simulate(sim_state, sim_ctrl, dt);
        auto sim_end = std::chrono::high_resolution_clock::now();
        
        auto sim_duration = std::chrono::duration_cast<std::chrono::microseconds>(sim_end - sim_start);
        result.sim_step_time_ms += sim_duration.count() / 1000.0;
        
        if (!sim_result.has_value()) {
            result.error_message = "Sim step failed at step " + std::to_string(step) + ": " + 
                                   sim::to_string(sim_result.error());
            result.num_steps = step;
            return result;
        }
        
        // Convert back to Frenet
        current_state = sim_to_frenet_state(sim_result.value().first, track);
        
        // Store
        result.frenet_states.push_back(current_state);
        result.time_stamps.push_back((step + 1) * dt);
    }
    
    auto end_total = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_total - start_total);
    result.total_time_ms = total_duration.count();
    
    result.success = true;
    result.num_steps = num_steps;
    
    // Convert to Cartesian for visualization
    auto [cart_X, cart_Y] = frenet_to_cartesian_trajectory(result.frenet_states, track);
    result.cart_X = std::move(cart_X);
    result.cart_Y = std::move(cart_Y);
    
    // Update timing statistics
    last_total_time_ = result.total_time_ms;
    total_sim_time_ += result.sim_step_time_ms;
    num_runs_++;
    
    return result;
}

double SimWrapper::last_total_time_ms() const {
    return last_total_time_;
}

double SimWrapper::average_sim_step_time_ms() const {
    if (num_runs_ == 0) return 0.0;
    return total_sim_time_ / static_cast<double>(num_runs_);
}

size_t SimWrapper::total_runs() const {
    return num_runs_;
}

void SimWrapper::reset_timing_stats() {
    last_total_time_ = 0.0;
    total_sim_time_ = 0.0;
    num_runs_ = 0;
}

sim::Sim::State SimWrapper::frenet_to_sim_state(
    const control::HighLevelController::State& frenet_state,
    const common::Track& track
) {
    common::FrenetPose frenet_pose = {frenet_state.s, frenet_state.n, frenet_state.psi};
    auto result = track.frenet_to_cartesian(frenet_pose);
    
    sim::Sim::State sim_state;
    sim_state.X = result.first.X;
    sim_state.Y = result.first.Y;
    sim_state.phi = result.first.phi;
    sim_state.v_x = frenet_state.v * std::cos(frenet_state.psi);
    sim_state.v_y = frenet_state.v * std::sin(frenet_state.psi);
    sim_state.omega = 0.0;  // Approximation
    sim_state.delta = frenet_state.delta;
    sim_state.tau_FL = 0.0;
    sim_state.tau_FR = 0.0;
    sim_state.tau_RL = 0.0;
    sim_state.tau_RR = 0.0;
    
    return sim_state;
}

control::HighLevelController::State SimWrapper::sim_to_frenet_state(
    const sim::Sim::State& sim_state,
    const common::Track& track
) {
    common::CartesianPose cartesian_pose = {sim_state.X, sim_state.Y, sim_state.phi};
    auto result = track.cartesian_to_frenet(cartesian_pose);
    
    control::HighLevelController::State frenet_state;
    frenet_state.s = result.first.s;
    frenet_state.n = result.first.n;
    frenet_state.psi = result.first.psi;
    frenet_state.v = std::sqrt(sim_state.v_x * sim_state.v_x + sim_state.v_y * sim_state.v_y);
    frenet_state.delta = sim_state.delta;
    
    return frenet_state;
}

std::pair<std::vector<double>, std::vector<double>> SimWrapper::frenet_to_cartesian_trajectory(
    const std::vector<control::HighLevelController::State>& states,
    const common::Track& track
) {
    std::vector<double> cart_X;
    std::vector<double> cart_Y;
    
    cart_X.reserve(states.size());
    cart_Y.reserve(states.size());
    
    for (const auto& state : states) {
        common::FrenetPose frenet_pose = {state.s, state.n, state.psi};
        auto result = track.frenet_to_cartesian(frenet_pose);
        
        cart_X.push_back(result.first.X);
        cart_Y.push_back(result.first.Y);
    }
    
    return {std::move(cart_X), std::move(cart_Y)};
}

}  // namespace tools
}  // namespace brains2
