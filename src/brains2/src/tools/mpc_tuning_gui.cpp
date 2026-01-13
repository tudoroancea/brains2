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
#include <limits>
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
        SDL_CreateWindow("MPC Tuning GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1600, 900, window_flags);
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
    brains2::tools::MPCResult last_hlc_result;
    brains2::tools::OpenLoopSimResult last_sim_result;
    bool mpc_configured = false;
    bool auto_recompute = true;
    bool run_simulation = true;
    int last_track_index = gui_state.selected_track_index;
    
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

    // Store previous parameters for change detection
    struct StoredParams {
        double v_ref = 3.0;
        double q_s = 10.0, q_n = 20.0, q_psi = 50.0, q_v = 20.0;
        double r_delta = 2.0, r_delta_dot = 1.0, r_tau = 0.0001;
        double q_s_f = 10000.0, q_n_f = 20000.0, q_psi_f = 50000.0, q_v_f = 20000.0;
        double v_max = 10.0;
        double delta_max = 0.5, delta_dot_max = 1.0;
        double tau_max = 200.0;
        double car_width = 1.55;
        double init_s = 0.0;
        double init_n = 0.0;
        double init_psi = 0.0;
        double init_v = 3.0;
    };
    StoredParams last_params;

    // Debouncing
    double last_param_change_time = 0.0;
    const double debounce_delay = 0.3;
    bool parameters_changed = false;

    // Helper functions
    auto parameters_have_changed = [&]() -> bool {
        const double eps = 1e-6;
        return std::abs(param_v_ref - last_params.v_ref) > eps ||
               std::abs(param_q_s - last_params.q_s) > eps ||
               std::abs(param_q_n - last_params.q_n) > eps ||
               std::abs(param_q_psi - last_params.q_psi) > eps ||
               std::abs(param_q_v - last_params.q_v) > eps ||
               std::abs(param_r_delta - last_params.r_delta) > eps ||
               std::abs(param_r_delta_dot - last_params.r_delta_dot) > eps ||
               std::abs(param_r_tau - last_params.r_tau) > eps ||
               std::abs(param_q_s_f - last_params.q_s_f) > eps ||
               std::abs(param_q_n_f - last_params.q_n_f) > eps ||
               std::abs(param_q_psi_f - last_params.q_psi_f) > eps ||
               std::abs(param_q_v_f - last_params.q_v_f) > eps ||
               std::abs(param_v_max - last_params.v_max) > eps ||
               std::abs(param_delta_max - last_params.delta_max) > eps ||
               std::abs(param_delta_dot_max - last_params.delta_dot_max) > eps ||
               std::abs(param_tau_max - last_params.tau_max) > eps ||
               std::abs(param_car_width - last_params.car_width) > eps ||
               std::abs(gui_state.initial_s - last_params.init_s) > eps ||
               std::abs(gui_state.initial_n - last_params.init_n) > eps ||
               std::abs(gui_state.initial_psi - last_params.init_psi) > eps ||
               std::abs(gui_state.initial_v - last_params.init_v) > eps;
    };

    auto update_stored_params = [&]() {
        last_params.v_ref = param_v_ref;
        last_params.q_s = param_q_s;
        last_params.q_n = param_q_n;
        last_params.q_psi = param_q_psi;
        last_params.q_v = param_q_v;
        last_params.r_delta = param_r_delta;
        last_params.r_delta_dot = param_r_delta_dot;
        last_params.r_tau = param_r_tau;
        last_params.q_s_f = param_q_s_f;
        last_params.q_n_f = param_q_n_f;
        last_params.q_psi_f = param_q_psi_f;
        last_params.q_v_f = param_q_v_f;
        last_params.v_max = param_v_max;
        last_params.delta_max = param_delta_max;
        last_params.delta_dot_max = param_delta_dot_max;
        last_params.tau_max = param_tau_max;
        last_params.car_width = param_car_width;
        last_params.init_s = gui_state.initial_s;
        last_params.init_n = gui_state.initial_n;
        last_params.init_psi = gui_state.initial_psi;
        last_params.init_v = gui_state.initial_v;
    };

    auto configure_mpc = [&]() -> brains2::tools::MPCParameters {
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
        return params;
    };

    auto get_initial_state = [&]() -> brains2::control::HighLevelController::State {
        return brains2::control::HighLevelController::State{
            gui_state.initial_s,
            gui_state.initial_n,
            gui_state.initial_psi,
            gui_state.initial_v,
            0.0
        };
    };

    auto run_computations = [&]() {
        if (!gui_state.current_track) return;
        
        brains2::tools::MPCParameters params = configure_mpc();
        last_hlc_result = mpc_wrapper.solve_open_loop(get_initial_state(), *gui_state.current_track);
        
        if (run_simulation) {
            brains2::tools::SimWrapper sim_wrapper;
            brains2::tools::SimParameters sim_params;
            sim_wrapper.configure(sim_params);
            last_sim_result = sim_wrapper.run_open_loop_simulation(
                get_initial_state(), *gui_state.current_track, params, last_hlc_result.control_trajectory);
        } else {
            last_sim_result.success = false;
            last_sim_result.cart_X.clear();
            last_sim_result.cart_Y.clear();
            last_sim_result.frenet_states.clear();
            last_sim_result.controls.clear();
            last_sim_result.time_stamps.clear();
        }
        
        update_stored_params();
        parameters_changed = false;
    };

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

        // Initial setup on first frame
        if (!mpc_configured && gui_state.current_track) {
            mpc_wrapper.configure(brains2::tools::MPCParameters());
            mpc_configured = mpc_wrapper.is_configured();
            if (mpc_configured) {
                run_computations();
            }
        }

        // Auto-recompute logic
        if (auto_recompute && gui_state.current_track && mpc_configured) {
            if (parameters_have_changed()) {
                double current_time = ImGui::GetTime();
                if (!parameters_changed) {
                    parameters_changed = true;
                    last_param_change_time = current_time;
                } else if (current_time - last_param_change_time > debounce_delay) {
                    run_computations();
                }
            }
        }

        // Get window size for responsive layout
        int win_w, win_h;
        SDL_GetWindowSize(window, &win_w, &win_h);
        float display_w = static_cast<float>(win_w);
        float display_h = static_cast<float>(win_h);

        // Layout constants
        const float controls_width = 300.0f;
        const float margin = 5.0f;
        const float plots_start_x = controls_width + margin;
        const float plots_width = display_w - plots_start_x - margin;
        const float track_height = display_h * 0.5f;
        const float state_plots_height = display_h - track_height - margin;
        const float plot_cols = 3.0f;
        const float plot_rows = 2.0f;
        const float state_plot_w = (plots_width - margin * (plot_cols - 1)) / plot_cols;
        const float state_plot_h = (state_plots_height - margin * (plot_rows - 1)) / plot_rows;

        double dt = 0.05;
        double max_time = brains2::tools::MPCWrapper::HORIZON_NF * dt;

        // ==================== CONTROLS PANEL ====================
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(controls_width, display_h));
        ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
        
        // Track selection
        ImGui::Text("Track Selection");
        ImGui::Separator();
        
        if (gui_state.available_tracks.size() > 0) {
            const char* track_names[10];
            for (size_t i = 0; i < gui_state.available_tracks.size(); i++) {
                track_names[i] = gui_state.available_tracks[i].name.c_str();
            }
            
            ImGui::SetNextItemWidth(-1);
            ImGui::Combo("##Track", &gui_state.selected_track_index, track_names, 
                        static_cast<int>(gui_state.available_tracks.size()));
            
            if (gui_state.selected_track_index != last_track_index) {
                last_track_index = gui_state.selected_track_index;
                gui_state.select_track(gui_state.selected_track_index);
                mpc_wrapper.configure(brains2::tools::MPCParameters());
                mpc_configured = mpc_wrapper.is_configured();
                run_computations();
            }
            
            if (gui_state.current_track) {
                ImGui::Text("Length: %.1f m | Horizon: %zu", 
                           gui_state.current_track->length(), 
                           brains2::tools::MPCWrapper::HORIZON_NF);
            }
        }
        
        ImGui::Spacing();
        
        // Auto-recompute and Sim control
        ImGui::Checkbox("Auto##recompute", &auto_recompute);
        ImGui::SameLine();
        ImGui::Checkbox("Sim", &run_simulation);
        ImGui::SameLine();
        if (ImGui::Button("Run")) {
            if (gui_state.current_track && mpc_configured) {
                run_computations();
            }
        }
        
        // Timing info
        ImGui::Spacing();
        if (last_hlc_result.success) {
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), "HLC: %.1f ms", last_hlc_result.solve_time_ms);
        }
        if (run_simulation && last_sim_result.success) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), "Sim: %.1f ms", last_sim_result.total_time_ms);
        }
        
        ImGui::Spacing();
        ImGui::Separator();
        
        // Initial position
        ImGui::Text("Initial State");
        ImGui::Separator();
        
        if (gui_state.current_track) {
            double min_s = gui_state.current_track->s_min();
            double max_s = min_s + gui_state.current_track->length() - 1.0;
            
            ImGui::SetNextItemWidth(-1);
            if (ImGui::SliderScalar("s##init", ImGuiDataType_Double, &gui_state.initial_s, &min_s, &max_s, "s: %.1f m")) {
                gui_state.update_initial_s(gui_state.initial_s);
                if (auto_recompute && mpc_configured) {
                    parameters_changed = true;
                    last_param_change_time = ImGui::GetTime();
                }
            }
            
            double n_min = -2.0, n_max = 2.0;
            ImGui::SetNextItemWidth(-1);
            if (ImGui::SliderScalar("n##init", ImGuiDataType_Double, &gui_state.initial_n, &n_min, &n_max, "n: %.2f m")) {
                if (auto_recompute && mpc_configured) {
                    parameters_changed = true;
                    last_param_change_time = ImGui::GetTime();
                }
            }
            
            double psi_min = -M_PI, psi_max = M_PI;
            ImGui::SetNextItemWidth(-1);
            if (ImGui::SliderScalar("psi##init", ImGuiDataType_Double, &gui_state.initial_psi, &psi_min, &psi_max, "psi: %.2f rad")) {
                if (auto_recompute && mpc_configured) {
                    parameters_changed = true;
                    last_param_change_time = ImGui::GetTime();
                }
            }
            
            double v_min = 0.0, v_max = 10.0;
            ImGui::SetNextItemWidth(-1);
            if (ImGui::SliderScalar("v##init", ImGuiDataType_Double, &gui_state.initial_v, &v_min, &v_max, "v: %.1f m/s")) {
                if (auto_recompute && mpc_configured) {
                    parameters_changed = true;
                    last_param_change_time = ImGui::GetTime();
                }
            }
        }
        
        ImGui::Spacing();

        // Cost Parameters
        if (ImGui::CollapsingHeader("Cost Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
            double v_ref_min = 0.5, v_ref_max = 10.0;
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("v_ref", ImGuiDataType_Double, &param_v_ref, &v_ref_min, &v_ref_max, "v_ref: %.1f");

            double q_min = 0.0, q_max = 200.0;
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("q_s", ImGuiDataType_Double, &param_q_s, &q_min, &q_max, "q_s: %.0f");
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("q_n", ImGuiDataType_Double, &param_q_n, &q_min, &q_max, "q_n: %.0f");
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("q_psi", ImGuiDataType_Double, &param_q_psi, &q_min, &q_max, "q_psi: %.0f");
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("q_v", ImGuiDataType_Double, &param_q_v, &q_min, &q_max, "q_v: %.0f");

            double r_min = 0.0, r_max = 20.0;
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("r_delta", ImGuiDataType_Double, &param_r_delta, &r_min, &r_max, "r_delta: %.1f");
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("r_delta_dot", ImGuiDataType_Double, &param_r_delta_dot, &r_min, &r_max, "r_d_dot: %.1f");
            
            double r_tau_min = 0.0, r_tau_max = 0.01;
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("r_tau", ImGuiDataType_Double, &param_r_tau, &r_tau_min, &r_tau_max, "r_tau: %.4f");

            if (ImGui::Button("Reset Cost")) {
                param_v_ref = 3.0; param_q_s = 10.0; param_q_n = 20.0; param_q_psi = 50.0; param_q_v = 20.0;
                param_r_delta = 2.0; param_r_delta_dot = 1.0; param_r_tau = 0.0001;
                param_q_s_f = 10000.0; param_q_n_f = 20000.0; param_q_psi_f = 50000.0; param_q_v_f = 20000.0;
            }
        }

        // Constraint Parameters
        if (ImGui::CollapsingHeader("Constraints", ImGuiTreeNodeFlags_DefaultOpen)) {
            double v_max_min = 1.0, v_max_max = 20.0;
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("v_max", ImGuiDataType_Double, &param_v_max, &v_max_min, &v_max_max, "v_max: %.1f");

            double delta_max_min = 0.1, delta_max_max = 1.0;
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("delta_max", ImGuiDataType_Double, &param_delta_max, &delta_max_min, &delta_max_max, "d_max: %.2f");

            double delta_dot_max_min = 0.1, delta_dot_max_max = 5.0;
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("delta_dot_max", ImGuiDataType_Double, &param_delta_dot_max, &delta_dot_max_min, &delta_dot_max_max, "d_dot: %.1f");

            double tau_max_min = 10.0, tau_max_max = 500.0;
            ImGui::SetNextItemWidth(-1);
            ImGui::SliderScalar("tau_max", ImGuiDataType_Double, &param_tau_max, &tau_max_min, &tau_max_max, "tau: %.0f");

            if (ImGui::Button("Reset Constraints")) {
                param_v_max = 10.0; param_delta_max = 0.5; param_delta_dot_max = 1.0; param_tau_max = 200.0;
            }
        }
        
        ImGui::End();

        // ==================== TRACK VIEW ====================
        ImGui::SetNextWindowPos(ImVec2(plots_start_x, 0));
        ImGui::SetNextWindowSize(ImVec2(plots_width, track_height));
        ImGui::Begin("Track View", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

        bool has_valid_track = gui_state.current_track &&
                               gui_state.selected_track_index >= 0 &&
                               gui_state.selected_track_index < static_cast<int>(gui_state.available_tracks.size());

        if (has_valid_track && ImPlot::BeginPlot("##Track", ImVec2(-1, -1), ImPlotFlags_Equal)) {
            // Compute bounds from trajectories only
            double min_x = 0, max_x = 0, min_y = 0, max_y = 0;
            bool has_bounds = false;

            auto update_bounds = [&](const std::vector<double>& xs, const std::vector<double>& ys) {
                for (size_t i = 0; i < xs.size() && i < ys.size(); i++) {
                    if (!has_bounds) {
                        min_x = max_x = xs[i];
                        min_y = max_y = ys[i];
                        has_bounds = true;
                    } else {
                        min_x = std::min(min_x, xs[i]);
                        max_x = std::max(max_x, xs[i]);
                        min_y = std::min(min_y, ys[i]);
                        max_y = std::max(max_y, ys[i]);
                    }
                }
            };

            if (last_hlc_result.success && !last_hlc_result.cart_X.empty()) {
                update_bounds(last_hlc_result.cart_X, last_hlc_result.cart_Y);
            }
            if (last_sim_result.success && !last_sim_result.cart_X.empty()) {
                update_bounds(last_sim_result.cart_X, last_sim_result.cart_Y);
            }

            // Setup axes BEFORE plotting
            ImPlot::SetupAxes("X (m)", "Y (m)");
            if (has_bounds) {
                double padding = std::max(max_x - min_x, max_y - min_y) * 0.2;
                if (padding < 2.0) padding = 2.0;
                ImPlot::SetupAxisLimits(ImAxis_X1, min_x - padding, max_x + padding, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, min_y - padding, max_y + padding, ImGuiCond_Always);
            }

            // Plot track
            const auto& track_info = gui_state.available_tracks[gui_state.selected_track_index];
            if (!track_info.X_vals.empty()) {
                ImPlot::PlotLine("Centerline", track_info.X_vals.data(), track_info.Y_vals.data(), 
                                static_cast<int>(track_info.X_vals.size()));
            }
            if (!track_info.left_bound_X.empty()) {
                ImPlot::PlotLine("Left", track_info.left_bound_X.data(), track_info.left_bound_Y.data(), 
                                static_cast<int>(track_info.left_bound_X.size()));
                ImPlot::PlotLine("Right", track_info.right_bound_X.data(), track_info.right_bound_Y.data(), 
                                static_cast<int>(track_info.right_bound_X.size()));
            }

            // Plot trajectories
            if (last_hlc_result.success && !last_hlc_result.cart_X.empty()) {
                ImPlot::PlotLine("HLC", last_hlc_result.cart_X.data(), last_hlc_result.cart_Y.data(), 
                                static_cast<int>(last_hlc_result.cart_X.size()));
            }
            if (run_simulation && last_sim_result.success && !last_sim_result.cart_X.empty()) {
                ImPlot::PlotLine("Sim", last_sim_result.cart_X.data(), last_sim_result.cart_Y.data(), 
                                static_cast<int>(last_sim_result.cart_X.size()));
            }

            // Start marker
            if (gui_state.current_track) {
                double start_X = gui_state.current_track->eval_X(gui_state.initial_s);
                double start_Y = gui_state.current_track->eval_Y(gui_state.initial_s);
                ImPlot::PlotScatter("Start", &start_X, &start_Y, 1);
            }

            ImPlot::EndPlot();
        }
        ImGui::End();

        // ==================== STATE PLOTS (2 rows x 3 columns) ====================
        // Helper lambda for state plots
        auto make_state_plot = [&](const char* title, int row, int col, 
                                   auto get_hlc_data, auto get_sim_data, const char* ylabel) {
            float x = plots_start_x + col * (state_plot_w + margin);
            float y = track_height + row * (state_plot_h + margin);
            
            ImGui::SetNextWindowPos(ImVec2(x, y));
            ImGui::SetNextWindowSize(ImVec2(state_plot_w, state_plot_h));
            ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
            
            auto [hlc_times, hlc_vals] = get_hlc_data();
            auto [sim_times, sim_vals] = get_sim_data();
            
            // Compute Y axis limits before plotting
            double min_val = std::numeric_limits<double>::max();
            double max_val = std::numeric_limits<double>::lowest();
            if (!hlc_vals.empty() || !sim_vals.empty()) {
                for (const auto& v : hlc_vals) {
                    min_val = std::min(min_val, v);
                    max_val = std::max(max_val, v);
                }
                for (const auto& v : sim_vals) {
                    min_val = std::min(min_val, v);
                    max_val = std::max(max_val, v);
                }
                double padding = std::max(max_val - min_val, 1.0) * 0.1;
                min_val -= padding;
                max_val += padding;
            }
            
            if (ImPlot::BeginPlot(title, ImVec2(-1, -1))) {
                ImPlot::SetupAxes("t (s)", ylabel);
                ImPlot::SetupAxisLimits(ImAxis_X1, 0, max_time, ImGuiCond_Always);
                if (min_val != std::numeric_limits<double>::max()) {
                    ImPlot::SetupAxisLimits(ImAxis_Y1, min_val, max_val, ImGuiCond_Always);
                }
                
                if (!hlc_vals.empty()) {
                    ImPlot::PlotLine("HLC", hlc_times.data(), hlc_vals.data(), static_cast<int>(hlc_vals.size()));
                }
                if (run_simulation && !sim_vals.empty()) {
                    ImPlot::PlotLine("Sim", sim_times.data(), sim_vals.data(), static_cast<int>(sim_vals.size()));
                }
                
                ImPlot::EndPlot();
            }
            ImGui::End();
        };

        // s vs time
        make_state_plot("s", 0, 0,
            [&]() {
                std::vector<double> times, vals;
                if (last_hlc_result.success) {
                    times = last_hlc_result.time_stamps;
                    for (const auto& s : last_hlc_result.state_trajectory) vals.push_back(s.s);
                }
                return std::make_pair(times, vals);
            },
            [&]() {
                std::vector<double> times, vals;
                if (last_sim_result.success) {
                    times = last_sim_result.time_stamps;
                    for (const auto& s : last_sim_result.frenet_states) vals.push_back(s.s);
                }
                return std::make_pair(times, vals);
            },
            "s (m)");

        // n vs time
        make_state_plot("n", 0, 1,
            [&]() {
                std::vector<double> times, vals;
                if (last_hlc_result.success) {
                    times = last_hlc_result.time_stamps;
                    for (const auto& s : last_hlc_result.state_trajectory) vals.push_back(s.n);
                }
                return std::make_pair(times, vals);
            },
            [&]() {
                std::vector<double> times, vals;
                if (last_sim_result.success) {
                    times = last_sim_result.time_stamps;
                    for (const auto& s : last_sim_result.frenet_states) vals.push_back(s.n);
                }
                return std::make_pair(times, vals);
            },
            "n (m)");

        // psi vs time
        make_state_plot("psi", 0, 2,
            [&]() {
                std::vector<double> times, vals;
                if (last_hlc_result.success) {
                    times = last_hlc_result.time_stamps;
                    for (const auto& s : last_hlc_result.state_trajectory) vals.push_back(s.psi);
                }
                return std::make_pair(times, vals);
            },
            [&]() {
                std::vector<double> times, vals;
                if (last_sim_result.success) {
                    times = last_sim_result.time_stamps;
                    for (const auto& s : last_sim_result.frenet_states) vals.push_back(s.psi);
                }
                return std::make_pair(times, vals);
            },
            "psi (rad)");

        // v vs time
        make_state_plot("v", 1, 0,
            [&]() {
                std::vector<double> times, vals;
                if (last_hlc_result.success) {
                    times = last_hlc_result.time_stamps;
                    for (const auto& s : last_hlc_result.state_trajectory) vals.push_back(s.v);
                }
                return std::make_pair(times, vals);
            },
            [&]() {
                std::vector<double> times, vals;
                if (last_sim_result.success) {
                    times = last_sim_result.time_stamps;
                    for (const auto& s : last_sim_result.frenet_states) vals.push_back(s.v);
                }
                return std::make_pair(times, vals);
            },
            "v (m/s)");

        // delta vs time
        make_state_plot("delta", 1, 1,
            [&]() {
                std::vector<double> times, vals;
                if (last_hlc_result.success) {
                    times = last_hlc_result.time_stamps;
                    for (const auto& s : last_hlc_result.state_trajectory) vals.push_back(s.delta);
                }
                return std::make_pair(times, vals);
            },
            [&]() {
                std::vector<double> times, vals;
                if (last_sim_result.success) {
                    times = last_sim_result.time_stamps;
                    for (const auto& s : last_sim_result.frenet_states) vals.push_back(s.delta);
                }
                return std::make_pair(times, vals);
            },
            "delta (rad)");

        // T vs time
        make_state_plot("T", 1, 2,
            [&]() {
                std::vector<double> times, vals;
                if (last_hlc_result.success) {
                    for (size_t i = 0; i < last_hlc_result.control_trajectory.size(); i++) {
                        times.push_back(i * dt);
                        vals.push_back(last_hlc_result.control_trajectory[i].tau);
                    }
                }
                return std::make_pair(times, vals);
            },
            [&]() {
                std::vector<double> times, vals;
                if (last_sim_result.success) {
                    for (size_t i = 0; i < last_sim_result.controls.size(); i++) {
                        times.push_back(i * dt);
                        vals.push_back(last_sim_result.controls[i].tau);
                    }
                }
                return std::make_pair(times, vals);
            },
            "T (Nm)");

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
