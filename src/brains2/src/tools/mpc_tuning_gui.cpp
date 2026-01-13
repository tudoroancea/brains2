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

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <SDL.h>
#include <SDL_opengl.h>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include "gui_state.hpp"
#include "mpc_wrapper.hpp"
#include "sim_wrapper.hpp"

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        printf("Error: %s\n", SDL_GetError());
        return -1;
    }

    const char* glsl_version = "#version 150";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    SDL_WindowFlags window_flags =
        (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window =
        SDL_CreateWindow("MPC Tuning GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1600, 1000, window_flags);
    if (window == nullptr) {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return -1;
    }

    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.1f, 0.1f, 0.1f, 1.00f);

    // Initialize GUI state
    brains2::tools::GuiState gui_state;
    
    // Initialize MPC wrapper
    brains2::tools::MPCWrapper mpc_wrapper;
    brains2::tools::MPCResult last_mpc_result;
    brains2::tools::ClosedLoopResult last_cl_result;
    bool mpc_configured = false;
    bool auto_recompute = false;
    
    // Cost parameter sliders
    double param_v_ref = 3.0;
    double param_q_s = 10.0;
    double param_q_n = 20.0;
    double param_q_psi = 50.0;
    double param_q_v = 20.0;
    double param_r_delta = 2.0;
    double param_r_delta_dot = 1.0;
    double param_r_tau = 0.0001;
    double param_q_s_f = 10000.0;
    double param_q_n_f = 20000.0;
    double param_q_psi_f = 50000.0;
    double param_q_v_f = 20000.0;

    // Constraint parameter sliders
    double param_v_max = 10.0;
    double param_delta_max = 0.5;
    double param_delta_dot_max = 1.0;
    double param_tau_max = 200.0;
    double param_car_width = 1.55;

    bool done = false;
    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) done = true;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window))
                done = true;
        }

        if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) {
            SDL_Delay(10);
            continue;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Main window with controls on the left
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(350, 1000), ImGuiCond_FirstUseEver);
        ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoResize);
        
        // Track selection
        ImGui::Text("Track Selection");
        ImGui::Separator();
        
        if (gui_state.available_tracks.size() > 0) {
            const char* track_names[10];
            for (size_t i = 0; i < gui_state.available_tracks.size(); i++) {
                track_names[i] = gui_state.available_tracks[i].name.c_str();
            }
            
            ImGui::Combo("Track", &gui_state.selected_track_index, track_names, 
                        static_cast<int>(gui_state.available_tracks.size()));
            
            if (ImGui::Button("Load Track")) {
                gui_state.select_track(gui_state.selected_track_index);
                mpc_wrapper.configure(brains2::tools::MPCParameters());
                mpc_configured = mpc_wrapper.is_configured();
            }
            
            if (gui_state.current_track) {
                ImGui::Text("Track length: %.2f m", gui_state.current_track->length());
            }
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "No tracks loaded!");
        }
        
        ImGui::Spacing();
        ImGui::Spacing();
        
        // Initial position
        ImGui::Text("Initial Position");
        ImGui::Separator();
        
        if (gui_state.current_track) {
            double min_s = gui_state.current_track->s_min();
            double max_s = min_s + gui_state.current_track->length() - 1.0;
            
            ImGui::Text("s (progress along track)");
            if (ImGui::SliderScalar("##initial_s", ImGuiDataType_Double, &gui_state.initial_s, &min_s, &max_s, "%.2f m")) {
                gui_state.update_initial_s(gui_state.initial_s);
            }
            
            ImGui::Text("n (lateral deviation)");
            double n_min = -2.0, n_max = 2.0;
            ImGui::SliderScalar("##initial_n", ImGuiDataType_Double, &gui_state.initial_n, &n_min, &n_max, "%.2f m");
            
            ImGui::Text("psi (heading error)");
            double psi_min = -M_PI, psi_max = M_PI;
            ImGui::SliderScalar("##initial_psi", ImGuiDataType_Double, &gui_state.initial_psi, &psi_min, &psi_max, "%.2f rad");
            
            ImGui::Text("v (velocity)");
            double v_min = 0.0, v_max = 10.0;
            ImGui::SliderScalar("##initial_v", ImGuiDataType_Double, &gui_state.initial_v, &v_min, &v_max, "%.2f m/s");
        }
        
        ImGui::Spacing();
        ImGui::Spacing();

        // MPC Cost Parameters - Collapsible section
        if (ImGui::CollapsingHeader("Cost Parameters##header", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Indent();

            ImGui::Text("Reference Velocity");
            double v_ref_min = 0.5, v_ref_max = 10.0;
            ImGui::SliderScalar("v_ref##cost", ImGuiDataType_Double, &param_v_ref, &v_ref_min, &v_ref_max, "%.2f m/s");

            ImGui::Spacing();
            ImGui::Text("State Weights (q_*)");
            ImGui::Separator();

            double q_min = 0.0, q_max = 200.0;
            ImGui::SliderScalar("q_s##cost", ImGuiDataType_Double, &param_q_s, &q_min, &q_max, "%.1f");
            ImGui::SliderScalar("q_n##cost", ImGuiDataType_Double, &param_q_n, &q_min, &q_max, "%.1f");
            ImGui::SliderScalar("q_psi##cost", ImGuiDataType_Double, &param_q_psi, &q_min, &q_max, "%.1f");
            ImGui::SliderScalar("q_v##cost", ImGuiDataType_Double, &param_q_v, &q_min, &q_max, "%.1f");

            ImGui::Spacing();
            ImGui::Text("Control Weights (r_*)");
            ImGui::Separator();

            double r_delta_min = 0.0, r_delta_max = 20.0;
            ImGui::SliderScalar("r_delta##cost", ImGuiDataType_Double, &param_r_delta, &r_delta_min, &r_delta_max, "%.1f");

            double r_delta_dot_min = 0.0, r_delta_dot_max = 10.0;
            ImGui::SliderScalar("r_delta_dot##cost", ImGuiDataType_Double, &param_r_delta_dot, &r_delta_dot_min, &r_delta_dot_max, "%.1f");

            double r_tau_min = 0.0, r_tau_max = 0.01;
            ImGui::SliderScalar("r_tau##cost", ImGuiDataType_Double, &param_r_tau, &r_tau_min, &r_tau_max, "%.5f");

            ImGui::Spacing();
            ImGui::Text("Terminal Weights (q_*_f)");
            ImGui::Separator();

            double q_f_min = 0.0, q_f_max = 100000.0;
            ImGui::SliderScalar("q_s_f##cost", ImGuiDataType_Double, &param_q_s_f, &q_f_min, &q_f_max, "%.0f");
            ImGui::SliderScalar("q_n_f##cost", ImGuiDataType_Double, &param_q_n_f, &q_f_min, &q_f_max, "%.0f");
            ImGui::SliderScalar("q_psi_f##cost", ImGuiDataType_Double, &param_q_psi_f, &q_f_min, &q_f_max, "%.0f");
            ImGui::SliderScalar("q_v_f##cost", ImGuiDataType_Double, &param_q_v_f, &q_f_min, &q_f_max, "%.0f");

            ImGui::Spacing();
            if (ImGui::Button("Reset Cost Defaults##button")) {
                param_v_ref = 3.0;
                param_q_s = 10.0;
                param_q_n = 20.0;
                param_q_psi = 50.0;
                param_q_v = 20.0;
                param_r_delta = 2.0;
                param_r_delta_dot = 1.0;
                param_r_tau = 0.0001;
                param_q_s_f = 10000.0;
                param_q_n_f = 20000.0;
                param_q_psi_f = 50000.0;
                param_q_v_f = 20000.0;
            }

            ImGui::Unindent();
        }

        ImGui::Spacing();

        // MPC Constraint Parameters - Collapsible section
        if (ImGui::CollapsingHeader("Constraint Parameters##header", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Indent();

            ImGui::Text("Velocity Limits");
            ImGui::Separator();
            double v_max_min = 1.0, v_max_max = 20.0;
            ImGui::SliderScalar("v_max##constraints", ImGuiDataType_Double, &param_v_max, &v_max_min, &v_max_max, "%.1f m/s");

            ImGui::Spacing();
            ImGui::Text("Steering Limits");
            ImGui::Separator();
            double delta_max_min = 0.1, delta_max_max = 1.0;
            ImGui::SliderScalar("delta_max##constraints", ImGuiDataType_Double, &param_delta_max, &delta_max_min, &delta_max_max, "%.2f rad");

            double delta_dot_max_min = 0.1, delta_dot_max_max = 5.0;
            ImGui::SliderScalar("delta_dot_max##constraints", ImGuiDataType_Double, &param_delta_dot_max, &delta_dot_max_min, &delta_dot_max_max, "%.2f rad/s");

            ImGui::Spacing();
            ImGui::Text("Torque Limits");
            ImGui::Separator();
            double tau_max_min = 10.0, tau_max_max = 500.0;
            ImGui::SliderScalar("tau_max##constraints", ImGuiDataType_Double, &param_tau_max, &tau_max_min, &tau_max_max, "%.0f Nm");

            ImGui::Spacing();
            ImGui::Text("Vehicle Dimensions");
            ImGui::Separator();
            double car_width_min = 1.0, car_width_max = 2.5;
            ImGui::SliderScalar("car_width##constraints", ImGuiDataType_Double, &param_car_width, &car_width_min, &car_width_max, "%.2f m");

            ImGui::Spacing();
            if (ImGui::Button("Reset Constraint Defaults##button")) {
                param_v_max = 10.0;
                param_delta_max = 0.5;
                param_delta_dot_max = 1.0;
                param_tau_max = 200.0;
                param_car_width = 1.55;
            }

            ImGui::Unindent();
        }
        
        ImGui::Spacing();
        ImGui::Spacing();
        
        // MPC Controls
        ImGui::Text("MPC Controls");
        ImGui::Separator();
        
        ImGui::Checkbox("Auto-recompute", &auto_recompute);
        
        if (ImGui::Button("Solve Open-Loop MPC")) {
            if (gui_state.current_track && mpc_configured) {
                brains2::tools::MPCParameters params = mpc_wrapper.get_parameters();
                params.cost_params.v_ref = param_v_ref;
                params.cost_params.q_s = param_q_s;
                params.cost_params.q_n = param_q_n;
                params.cost_params.q_psi = param_q_psi;
                params.cost_params.q_v = param_q_v;
                params.cost_params.r_delta = param_r_delta;
                params.cost_params.r_delta_dot = param_r_delta_dot;
                params.cost_params.r_tau = param_r_tau;
                params.cost_params.q_s_f = param_q_s_f;
                params.cost_params.q_n_f = param_q_n_f;
                params.cost_params.q_psi_f = param_q_psi_f;
                params.cost_params.q_v_f = param_q_v_f;
                params.constraints_params.v_max = param_v_max;
                params.constraints_params.delta_max = param_delta_max;
                params.constraints_params.delta_dot_max = param_delta_dot_max;
                params.constraints_params.tau_max = param_tau_max;
                params.constraints_params.car_width = param_car_width;

                mpc_wrapper.configure(params);

                brains2::control::HighLevelController::State initial_state = {
                    gui_state.initial_s,
                    gui_state.initial_n,
                    gui_state.initial_psi,
                    gui_state.initial_v,
                    0.0  // initial delta
                };

                last_mpc_result = mpc_wrapper.solve_open_loop(initial_state, *gui_state.current_track);
            }
        }
        
        ImGui::Spacing();
        ImGui::Separator();
        
        // Closed-Loop Simulation
        ImGui::Text("Closed-Loop Simulation");
        ImGui::Separator();
        
        static int sim_steps = 100;  // Default 5 seconds
        ImGui::Text("Simulation steps (dt=0.05s)");
        ImGui::SliderInt("##sim_steps", &sim_steps, 10, 500);
        
        if (ImGui::Button("Run Closed-Loop Sim")) {
            if (gui_state.current_track) {
                brains2::tools::SimWrapper sim_wrapper;
                brains2::tools::SimParameters sim_params;
                sim_wrapper.configure(sim_params);

                brains2::tools::MPCParameters params = mpc_wrapper.get_parameters();
                params.cost_params.v_ref = param_v_ref;
                params.cost_params.q_s = param_q_s;
                params.cost_params.q_n = param_q_n;
                params.cost_params.q_psi = param_q_psi;
                params.cost_params.q_v = param_q_v;
                params.cost_params.r_delta = param_r_delta;
                params.cost_params.r_delta_dot = param_r_delta_dot;
                params.cost_params.r_tau = param_r_tau;
                params.cost_params.q_s_f = param_q_s_f;
                params.cost_params.q_n_f = param_q_n_f;
                params.cost_params.q_psi_f = param_q_psi_f;
                params.cost_params.q_v_f = param_q_v_f;
                params.constraints_params.v_max = param_v_max;
                params.constraints_params.delta_max = param_delta_max;
                params.constraints_params.delta_dot_max = param_delta_dot_max;
                params.constraints_params.tau_max = param_tau_max;
                params.constraints_params.car_width = param_car_width;

                mpc_wrapper.configure(params);

                brains2::control::HighLevelController::State initial_state = {
                    gui_state.initial_s,
                    gui_state.initial_n,
                    gui_state.initial_psi,
                    gui_state.initial_v,
                    0.0  // initial delta
                };

                last_cl_result = sim_wrapper.run_closed_loop_simulation(
                    initial_state, *gui_state.current_track, params, static_cast<size_t>(sim_steps));
            }
        }
        
        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::Button("Reset All Parameters")) {
            // Reset cost parameters
            param_v_ref = 3.0;
            param_q_s = 10.0;
            param_q_n = 20.0;
            param_q_psi = 50.0;
            param_q_v = 20.0;
            param_r_delta = 2.0;
            param_r_delta_dot = 1.0;
            param_r_tau = 0.0001;
            param_q_s_f = 10000.0;
            param_q_n_f = 20000.0;
            param_q_psi_f = 50000.0;
            param_q_v_f = 20000.0;

            // Reset constraint parameters
            param_v_max = 10.0;
            param_delta_max = 0.5;
            param_delta_dot_max = 1.0;
            param_tau_max = 200.0;
            param_car_width = 1.55;
        }

        ImGui::End();

        // Track visualization window
        ImGui::SetNextWindowPos(ImVec2(360, 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(800, 500), ImGuiCond_FirstUseEver);
        ImGui::Begin("Track Visualization");
        
        bool has_valid_track = gui_state.current_track && 
                               gui_state.selected_track_index >= 0 &&
                               gui_state.selected_track_index < static_cast<int>(gui_state.available_tracks.size());
        
        if (has_valid_track && ImPlot::BeginPlot("Track View", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("X (m)", "Y (m)");
            ImPlot::SetupAxisLimits(ImAxis_X1, -50, 50, ImGuiCond_Once);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -50, 50, ImGuiCond_Once);
            
            // Plot track centerline
            if (gui_state.show_centerline) {
                const auto& track_info = gui_state.available_tracks[gui_state.selected_track_index];
                if (!track_info.X_vals.empty()) {
                    ImPlot::PlotLine("Centerline", 
                                    track_info.X_vals.data(), 
                                    track_info.Y_vals.data(), 
                                    static_cast<int>(track_info.X_vals.size()));
                }
            }
            
            // Plot track bounds
            if (gui_state.show_track_bounds) {
                const auto& track_info = gui_state.available_tracks[gui_state.selected_track_index];
                if (!track_info.left_bound_X.empty()) {
                    ImPlot::PlotLine("Left Bound", 
                                    track_info.left_bound_X.data(), 
                                    track_info.left_bound_Y.data(), 
                                    static_cast<int>(track_info.left_bound_X.size()),
                                    ImPlotLineFlags_None);
                    ImPlot::PlotLine("Right Bound", 
                                    track_info.right_bound_X.data(), 
                                    track_info.right_bound_Y.data(), 
                                    static_cast<int>(track_info.right_bound_X.size()),
                                    ImPlotLineFlags_None);
                }
            }
            
            // Plot open-loop trajectory
            if (last_mpc_result.success && !last_mpc_result.cart_X.empty()) {
                ImPlot::PlotLine("Open-Loop Trajectory", 
                                last_mpc_result.cart_X.data(), 
                                last_mpc_result.cart_Y.data(), 
                                static_cast<int>(last_mpc_result.cart_X.size()));
            }
            
            // Plot closed-loop trajectory
            if (last_cl_result.success && !last_cl_result.cart_X.empty()) {
                ImPlot::PlotLine("Closed-Loop Trajectory", 
                                last_cl_result.cart_X.data(), 
                                last_cl_result.cart_Y.data(), 
                                static_cast<int>(last_cl_result.cart_X.size()));
            }
            
            // Plot start position
            if (gui_state.show_start_position && gui_state.current_track) {
                double start_X = gui_state.current_track->eval_X(gui_state.initial_s);
                double start_Y = gui_state.current_track->eval_Y(gui_state.initial_s);
                double start_phi = gui_state.current_track->eval_phi(gui_state.initial_s);
                
                // Draw a marker at start position
                ImPlot::PlotScatter("Start", &start_X, &start_Y, 1);
                
                // Draw heading arrow
                double arrow_length = 2.0;
                double arrow_X[2] = {start_X, start_X + arrow_length * std::cos(start_phi)};
                double arrow_Y[2] = {start_Y, start_Y + arrow_length * std::sin(start_phi)};
                ImPlot::PlotLine("Heading", arrow_X, arrow_Y, 2);
            }
            
            ImPlot::EndPlot();
        } else {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No track loaded");
        }
        
        ImGui::End();

        // State plots
        ImGui::SetNextWindowPos(ImVec2(1170, 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(420, 500), ImGuiCond_FirstUseEver);
        ImGui::Begin("State Plots");
        
        if (last_mpc_result.success && !last_mpc_result.state_trajectory.empty()) {
            // Extract state data
            std::vector<double> s_vals, n_vals, psi_vals, v_vals, delta_vals;
            for (const auto& state : last_mpc_result.state_trajectory) {
                s_vals.push_back(state.s);
                n_vals.push_back(state.n);
                psi_vals.push_back(state.psi);
                v_vals.push_back(state.v);
                delta_vals.push_back(state.delta);
            }
            
            if (ImPlot::BeginPlot("States vs Time", ImVec2(-1, 450))) {
                ImPlot::SetupAxes("Time (s)", "");
                double max_time = 0;
                if (!last_mpc_result.time_stamps.empty()) {
                    max_time = std::max(max_time, last_mpc_result.time_stamps.back());
                }
                if (!last_cl_result.time_stamps.empty()) {
                    max_time = std::max(max_time, last_cl_result.time_stamps.back());
                }
                if (max_time > 0) {
                    ImPlot::SetupAxisLimits(ImAxis_X1, 0, max_time, ImGuiCond_Always);
                }
                
                // Plot open-loop states
                if (last_mpc_result.success && !last_mpc_result.state_trajectory.empty()) {
                    ImPlot::PlotLine("s (OL)", last_mpc_result.time_stamps.data(), s_vals.data(), static_cast<int>(s_vals.size()));
                    ImPlot::PlotLine("n (OL)", last_mpc_result.time_stamps.data(), n_vals.data(), static_cast<int>(n_vals.size()));
                    ImPlot::PlotLine("psi (OL)", last_mpc_result.time_stamps.data(), psi_vals.data(), static_cast<int>(psi_vals.size()));
                    ImPlot::PlotLine("v (OL)", last_mpc_result.time_stamps.data(), v_vals.data(), static_cast<int>(v_vals.size()));
                    ImPlot::PlotLine("delta (OL)", last_mpc_result.time_stamps.data(), delta_vals.data(), static_cast<int>(delta_vals.size()));
                }
                
                // Plot closed-loop states
                if (last_cl_result.success && !last_cl_result.frenet_states.empty()) {
                    std::vector<double> cl_s, cl_n, cl_psi, cl_v, cl_delta;
                    cl_s.reserve(last_cl_result.frenet_states.size());
                    cl_n.reserve(last_cl_result.frenet_states.size());
                    cl_psi.reserve(last_cl_result.frenet_states.size());
                    cl_v.reserve(last_cl_result.frenet_states.size());
                    cl_delta.reserve(last_cl_result.frenet_states.size());
                    
                    for (const auto& state : last_cl_result.frenet_states) {
                        cl_s.push_back(state.s);
                        cl_n.push_back(state.n);
                        cl_psi.push_back(state.psi);
                        cl_v.push_back(state.v);
                        cl_delta.push_back(state.delta);
                    }
                    
                    ImPlot::PlotLine("s (CL)", last_cl_result.time_stamps.data(), cl_s.data(), static_cast<int>(cl_s.size()));
                    ImPlot::PlotLine("n (CL)", last_cl_result.time_stamps.data(), cl_n.data(), static_cast<int>(cl_n.size()));
                    ImPlot::PlotLine("psi (CL)", last_cl_result.time_stamps.data(), cl_psi.data(), static_cast<int>(cl_psi.size()));
                    ImPlot::PlotLine("v (CL)", last_cl_result.time_stamps.data(), cl_v.data(), static_cast<int>(cl_v.size()));
                    ImPlot::PlotLine("delta (CL)", last_cl_result.time_stamps.data(), cl_delta.data(), static_cast<int>(cl_delta.size()));
                }
                
                ImPlot::EndPlot();
            }
        } else {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No MPC solution available.\nClick 'Solve Open-Loop MPC' to compute.");
        }
        
        ImGui::End();

        // Status / Info window
        ImGui::SetNextWindowPos(ImVec2(360, 510), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(1230, 480), ImGuiCond_FirstUseEver);
        ImGui::Begin("Status");
        ImGui::Text("MPC Tuning GUI - Phase 5: Parameter Tuning Interface");
        ImGui::Separator();
        
        // MPC status
        if (mpc_configured) {
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), "MPC: Configured");
        } else {
            ImGui::TextColored(ImVec4(0.8f, 0.3f, 0.3f, 1.0f), "MPC: Not Configured");
        }
        
        if (last_mpc_result.success) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), " | Open-Loop: %.2f ms", last_mpc_result.solve_time_ms);
        } else if (last_mpc_result.solve_time_ms > 0) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.8f, 0.3f, 0.3f, 1.0f), " | Open-Loop failed");
        }
        
        // Closed-loop status
        if (last_cl_result.success) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), " | Closed-Loop: %.2f ms (%zu steps)", 
                              last_cl_result.total_time_ms, last_cl_result.num_steps);
        } else if (last_cl_result.total_time_ms > 0) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.8f, 0.3f, 0.3f, 1.0f), " | Closed-Loop failed: %s", 
                              last_cl_result.error_message.c_str());
        }
        
        ImGui::Separator();
        
        // Display current parameters
        ImGui::Text("Current Parameters:");
        ImGui::Columns(4, "params");
        ImGui::Separator();

        // Cost parameters
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Cost Parameters:");
        ImGui::NextColumn();
        ImGui::NextColumn();
        ImGui::NextColumn();
        ImGui::NextColumn();

        ImGui::Text("v_ref: %.2f m/s", param_v_ref);
        ImGui::NextColumn();
        ImGui::Text("q_s: %.1f", param_q_s);
        ImGui::NextColumn();
        ImGui::Text("q_n: %.1f", param_q_n);
        ImGui::NextColumn();
        ImGui::Text("q_psi: %.1f", param_q_psi);
        ImGui::NextColumn();
        ImGui::Text("q_v: %.1f", param_q_v);
        ImGui::NextColumn();
        ImGui::Text("r_delta: %.1f", param_r_delta);
        ImGui::NextColumn();
        ImGui::Text("r_delta_dot: %.1f", param_r_delta_dot);
        ImGui::NextColumn();
        ImGui::Text("r_tau: %.5f", param_r_tau);
        ImGui::NextColumn();
        ImGui::Text("q_s_f: %.0f", param_q_s_f);
        ImGui::NextColumn();
        ImGui::Text("q_n_f: %.0f", param_q_n_f);
        ImGui::NextColumn();
        ImGui::Text("q_psi_f: %.0f", param_q_psi_f);
        ImGui::NextColumn();
        ImGui::Text("q_v_f: %.0f", param_q_v_f);
        ImGui::NextColumn();

        // Constraint parameters
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.4f, 1.0f), "Constraints:");
        ImGui::NextColumn();
        ImGui::NextColumn();
        ImGui::NextColumn();
        ImGui::NextColumn();

        ImGui::Text("v_max: %.1f m/s", param_v_max);
        ImGui::NextColumn();
        ImGui::Text("delta_max: %.2f rad", param_delta_max);
        ImGui::NextColumn();
        ImGui::Text("delta_dot_max: %.2f rad/s", param_delta_dot_max);
        ImGui::NextColumn();
        ImGui::Text("tau_max: %.0f Nm", param_tau_max);
        ImGui::NextColumn();
        ImGui::Text("car_width: %.2f m", param_car_width);
        ImGui::NextColumn();
        ImGui::NextColumn();
        ImGui::NextColumn();
        ImGui::NextColumn();

        ImGui::Columns(1);
        
        ImGui::Separator();
        ImGui::TextWrapped("Instructions:");
        ImGui::BulletText("Adjust parameters using the sliders on the left");
        ImGui::BulletText("Click 'Solve Open-Loop MPC' for single-shot trajectory prediction");
        ImGui::BulletText("Click 'Run Closed-Loop Sim' for full MPC+Sim simulation");
        ImGui::BulletText("Open-loop: green, Closed-loop: yellow on track plot");
        ImGui::BulletText("State plots show OL (solid) and CL (dashed) trajectories");
        
        ImGui::End();

        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
