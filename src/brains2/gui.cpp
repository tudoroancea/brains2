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

#include <cmath>
#include <cstdio>
#include <memory>
#include "brains2/common/track.hpp"
#include "brains2/control/high_level_controller.hpp"
#include "brains2/control/low_level_controller.hpp"
#include "brains2/external/expected.hpp"
#include "brains2/sim/sim.hpp"
#include "imgui/backends/imgui_impl_sdl2.h"
#include "imgui/backends/imgui_impl_sdlrenderer2.h"
#include "imgui/imgui.h"
#include "imgui/implot.h"
#include "SDL.h"
#include "yaml-cpp/yaml.h"

namespace ImGui {
bool SliderDouble(const char* label,
                  double* v,
                  double v_min,
                  double v_max,
                  const char* format = "%.3f",
                  ImGuiSliderFlags flags = 0) {
    return SliderScalar(label, ImGuiDataType_Double, v, &v_min, &v_max, format, flags);
}
}  // namespace ImGui

#if !SDL_VERSION_ATLEAST(2, 0, 17)
#error This backend requires SDL 2.0.17+ because of SDL_RenderGeometry() function
#endif

#ifndef CAR_CONSTANTS_PATH
#error CAR_CONSTANTS_PATH is not defined
#endif

using brains2::common::Track;
using brains2::control::HLC;
using brains2::control::LLC;
using brains2::sim::Sim;

static tl::expected<Track, Track::Error> generate_constant_curvature_track(
    const double curvature = 0.0,
    const double s_max = 7.0,
    const double width = 1.5,
    const size_t N = 20) {
    Eigen::VectorXd s = Eigen::VectorXd::LinSpaced(N, 0.0, s_max);
    Eigen::VectorXd kappa = curvature * Eigen::VectorXd::Ones(N);
    Eigen::VectorXd w = width * Eigen::VectorXd::Ones(N);
    if (std::fabs(curvature) < 1e-6) {
        return Track::from_values(s,
                                  s,
                                  Eigen::VectorXd::Zero(N),
                                  Eigen::VectorXd::Zero(N),
                                  kappa,
                                  w);
    }
    const auto R = 1 / std::abs(curvature);
    const Eigen::VectorXd angles = (s / R).array() - M_PI_2;
    const Eigen::VectorXd X = R * angles.array().cos();
    const Eigen::VectorXd Y = (R * angles.array().sin() + R) * (curvature > 0.0 ? 1.0 : -1.0);
    const Eigen::VectorXd phi = (angles.array() + M_PI_2) * (curvature > 0.0 ? 1.0 : -1.0);
    return Track::from_values(s, X, Y, phi, kappa, w);
}

// Main code
int main(int, char**) {
    // Setup SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        printf("Error: %s\n", SDL_GetError());
        return -1;
    }

    // From 2.0.18: Enable native IME.
#ifdef SDL_HINT_IME_SHOW_UI
    SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif

    // Create window with SDL_Renderer graphics context
    SDL_WindowFlags window_flags =
        (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("Dear ImGui SDL2+SDL_Renderer example",
                                          SDL_WINDOWPOS_CENTERED,
                                          SDL_WINDOWPOS_CENTERED,
                                          1280,
                                          720,
                                          window_flags);
    if (window == nullptr) {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return -1;
    }
    SDL_Renderer* renderer =
        SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr) {
        SDL_Log("Error creating SDL_Renderer!");
        return -1;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple
    // fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the
    // font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those
    // errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture
    // when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below
    // will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher
    // quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    io.Fonts->AddFontDefault();

    // load yaml file with car constants
    YAML::Node car_constants = YAML::LoadFile(CAR_CONSTANTS_PATH);

    Sim sim(Sim::Parameters{car_constants["inertia"]["mass"].as<double>(),
                            car_constants["inertia"]["yaw_inertia"].as<double>(),
                            car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
                            car_constants["geometry"]["wheelbase"].as<double>() -
                                car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
                            car_constants["drivetrain"]["C_m0"].as<double>(),
                            car_constants["drivetrain"]["C_r0"].as<double>(),
                            car_constants["drivetrain"]["C_r1"].as<double>(),
                            car_constants["drivetrain"]["C_r2"].as<double>(),
                            car_constants["actuators"]["motor_time_constant"].as<double>(),
                            car_constants["actuators"]["steering_time_constant"].as<double>(),
                            car_constants["geometry"]["cog_height"].as<double>(),
                            car_constants["geometry"]["axle_track"].as<double>(),
                            car_constants["aero"]["C_downforce"].as<double>(),
                            car_constants["pacejka"]["constant"]["Ba"].as<double>(),
                            car_constants["pacejka"]["constant"]["Ca"].as<double>(),
                            car_constants["pacejka"]["constant"]["Da"].as<double>(),
                            car_constants["pacejka"]["constant"]["Ea"].as<double>()},
            Sim::Limits{car_constants["actuators"]["torque_max"].as<double>(),
                        car_constants["actuators"]["steering_max"].as<double>(),
                        car_constants["actuators"]["steering_rate_max"].as<double>()});

    static double K_tv = 300.0;
    const LLC::ModelParams llc_model_params{
        car_constants["inertia"]["mass"].as<double>(),
        car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
        car_constants["geometry"]["wheelbase"].as<double>() -
            car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
        car_constants["geometry"]["axle_track"].as<double>(),
        car_constants["geometry"]["cog_height"].as<double>(),
        car_constants["aero"]["C_downforce"].as<double>(),
        car_constants["actuators"]["torque_max"].as<double>(),
    };

    static size_t Nf = 20;
    const HLC::ModelParams hlc_model_params{
        1.0 / 20.0,
        car_constants["inertia"]["mass"].as<double>(),
        car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
        car_constants["geometry"]["wheelbase"].as<double>() -
            car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
        car_constants["drivetrain"]["C_m0"].as<double>(),
        car_constants["drivetrain"]["C_r0"].as<double>(),
        car_constants["drivetrain"]["C_r1"].as<double>(),
        car_constants["drivetrain"]["C_r2"].as<double>(),
        car_constants["actuators"]["steering_time_constant"].as<double>(),
    };
    static HLC::ConstraintsParams constraints_params{
        10.0,
        car_constants["actuators"]["steering_max"].as<double>(),
        car_constants["actuators"]["steering_rate_max"].as<double>(),
        car_constants["actuators"]["torque_max"].as<double>(),
        car_constants["geometry"]["car_width"].as<double>(),
    };
    static HLC::CostParams cost_params{
        5.0,
        10.0,
        20.0,
        50.0,
        20.0,
        2.0,
        1.0,
        0.001,
        10000.0,
        20000.0,
        50000.0,
        20000.0,
    };
    const HLC::SolverParams solver_params{false, "fatrop"};
    Sim::State initial_state{};
    static double curvature = 0.0;

    bool show_demo_window = true;

    // Main loop
    bool done = false;
    while (!done) {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui
        // wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main
        // application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main
        // application, or clear/overwrite your copy of the keyboard data. Generally you may always
        // pass all inputs to dear imgui, and hide them from your application based on those two
        // flags.
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                done = true;
            }
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window)) {
                done = true;
            }
        }
        if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) {
            SDL_Delay(10);
            continue;
        }

        // Start the Dear ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You
        // can browse its code to learn more about Dear ImGui!).
        if (show_demo_window) {
            ImGui::ShowDemoWindow(&show_demo_window);
        }

        // 2. config panel
        {
            ImGui::SetNextWindowSize(ImVec2(400, 600), ImGuiCond_FirstUseEver);
            ImGui::Begin("Config");

            ImGui::SeparatorText("Track");
            ImGui::SliderDouble("curvature", &curvature, -1.0 / 6.0, 1.0 / 6.0);

            ImGui::SeparatorText("Initial state");
            // TODO: adjust limits
            ImGui::SliderDouble("Y", &initial_state.Y, -2.0, 2.0);
            ImGui::SliderDouble("phi", &initial_state.phi, -2.0, 2.0);
            ImGui::SliderDouble("v_x", &initial_state.v_x, -2.0, 2.0);
            ImGui::SliderDouble("v_y", &initial_state.v_y, -2.0, 2.0);
            ImGui::SliderDouble("omega", &initial_state.omega, -2.0, 2.0);
            ImGui::SliderDouble("delta", &initial_state.delta, -2.0, 2.0);
            ImGui::SliderDouble("tau_FL", &initial_state.tau_FL, -2.0, 2.0);
            ImGui::SliderDouble("tau_FR", &initial_state.tau_FR, -2.0, 2.0);
            ImGui::SliderDouble("tau_RL", &initial_state.tau_RL, -2.0, 2.0);
            ImGui::SliderDouble("tau_RR", &initial_state.tau_RR, -2.0, 2.0);

            ImGui::SeparatorText("LLC params");
            ImGui::InputDouble("K_tv", &K_tv, 10.0, 100.0);

            // ImGui::SeparatorText("HLC params");
            // ImGui::InputScalar("Nf", ImGuiDataType_U32, &Nf);
            // ImGui::InputDouble("v_max", &constraints_params.v_max, 1.0, 10.0);
            // ImGui::InputDouble("v_ref", &cost_params.v_ref, 1.0, 10.0);
            // ImGui::InputDouble("q_s", &cost_params.q_s, 1.0, 10.0);
            // ImGui::InputDouble("q_n", &cost_params.q_n, 1.0, 10.0);
            // ImGui::InputDouble("q_psi", &cost_params.q_psi, 1.0, 10.0);
            // ImGui::InputDouble("q_v", &cost_params.q_v, 1.0, 10.0);
            // ImGui::InputDouble("r_delta", &cost_params.r_delta, 1.0, 10.0);
            // ImGui::InputDouble("r_delta_dot", &cost_params.r_delta_dot, 1.0, 10.0);
            // ImGui::InputDouble("r_tau", &cost_params.r_tau, 1.0, 10.0);
            // ImGui::InputDouble("q_s_f", &cost_params.q_s_f, 1.0, 10.0);
            // ImGui::InputDouble("q_n_f", &cost_params.q_n_f, 1.0, 10.0);
            // ImGui::InputDouble("q_psi_f", &cost_params.q_psi_f, 1.0, 10.0);
            // ImGui::InputDouble("q_v_f", &cost_params.q_v_f, 1.0, 10.0);

            ImGui::End();
        }

        // Update track
        auto track = generate_constant_curvature_track(curvature);

        // update the LLC and HLC
        LLC llc(K_tv, llc_model_params);
        // HLC hlc(Nf, hlc_model_params, constraints_params, cost_params, solver_params);

        // Compute the open-loop prediction of the HLC
        // const auto controls =
        //     hlc.compute_control(HLC::State{0.0,
        //                                    initial_state.Y,
        //                                    initial_state.phi,
        //                                    std::hypot(initial_state.v_x, initial_state.v_y),
        //                                    initial_state.delta},
        //                         *track);
        // if (!controls) { }

        // Compute the closed-loop response
        std::vector<double> X_ref(Nf), Y_ref(Nf), omega_ref(Nf);
        std::vector<double> X_cl(Nf), Y_cl(Nf), omega_cl(Nf);
        // omega_ref is constant (omega_kin)

        // Sim::State state{current_state};

        // 3. plot s, n, psi, v, delta, tau, tau_ij
        {
            ImGui::Begin("Trajectory visualization");
            if (ImPlot::BeginPlot("Trajectory")) {
                ImPlot::PlotLine("traj", X_cl.data(), Y_cl.data(), X_cl.size());
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot("Angular Velocity")) {
                ImPlot::PlotLine("Omega", omega_cl.data(), omega_cl.size());
                // ImPlot::PlotLineG(const char *label_id, ImPlotGetter getter, void *data, int
                // count)
                ImPlot::EndPlot();
            }
            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        ImVec4 clear_color{0.45f, 0.55f, 0.60f, 1.00f};
        SDL_SetRenderDrawColor(renderer,
                               (Uint8)(clear_color.x * 255),
                               (Uint8)(clear_color.y * 255),
                               (Uint8)(clear_color.z * 255),
                               (Uint8)(clear_color.w * 255));
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    // Cleanup
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
