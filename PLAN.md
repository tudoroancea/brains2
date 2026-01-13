# MPC Tuning GUI - Implementation Plan

## Overview
A C++ GUI application using imgui_bundle to compare how MPC controller tuning influences open-loop and closed-loop trajectories. The application will allow real-time parameter adjustment and display trajectory plots with computation timing.

## Technical Specifications

### Dependencies
- **imgui_bundle** (via CMake FetchContent) - includes Dear ImGui + ImPlot
- Existing **brains2** libraries: `brains2_control`, `brains2_sim`, `brains2_common`
- **Eigen3**, **CasADi**, **acados** (existing project dependencies)

### Key Components
1. **HighLevelController** (MPC) - from `brains2/control/high_level_controller.hpp`
2. **Sim** (Simulator) - from `brains2/sim/sim.hpp`
3. **Track** - from `brains2/common/track.hpp`

### Definitions
- **Open-loop**: Single MPC solve from initial state → displays predicted trajectory (x_opt, u_opt) over horizon Nf
- **Closed-loop**: Iterative MPC + Sim loop → applies first control, simulates, recomputes MPC

---

## Phase 1: Project Setup & Basic Window
> Set up build system and create a basic imgui window

- [x] **1.1** Create `src/brains2/src/tools/` directory for the tuning GUI
- [x] **1.2** Create `CMakeLists.txt` for the tuning tool with FetchContent for imgui_bundle
- [x] **1.3** Create basic `mpc_tuning_gui.cpp` with imgui_bundle hello world window
- [x] **1.4** Integrate into main `src/brains2/CMakeLists.txt` build system
- [x] **1.5** Verify the application builds and displays a window

---

## Phase 2: Track Loading & Visualization ✓ COMPLETE
> Load tracks and display them in the GUI

- [x] **2.1** Add track loading functionality (alpha, beta, gamma from track_database)
- [x] **2.2** Create ImPlot subplot for track visualization (X-Y plot with centerline)
- [x] **2.3** Add track selection dropdown/combo box
- [x] **2.4** Add initial position slider (s coordinate along track)
- [x] **2.5** Display track bounds (width) on the track plot

---

## Phase 3: Open-Loop MPC Integration ✓ COMPLETE
> Implement single-shot MPC solve and trajectory display

- [x] **3.1** Create wrapper class for MPC with configurable parameters
- [x] **3.2** Implement open-loop solve function (single MPC call)
- [x] **3.3** Add timing measurement for MPC solve
- [x] **3.4** Convert Frenet trajectory (s, n, psi, v, delta) to Cartesian (X, Y, phi) for plotting
- [x] **3.5** Display open-loop trajectory on track plot
- [x] **3.6** Create state plots (s, n, psi, v, delta vs time)
- [x] **3.7** Create control plots (u_delta, tau vs time)
- [x] **3.8** Display computation time in UI

---

## Phase 4: Closed-Loop Simulation Integration ✓ COMPLETE
> Implement MPC + Simulator loop

- [x] **4.1** Create wrapper for Sim class initialization
- [x] **4.2** Implement closed-loop simulation function:
  - Initialize state
  - Loop: MPC solve → extract first control → Sim step → repeat
  - Store full trajectory
- [x] **4.3** Add simulation duration/steps control
- [x] **4.4** Add timing measurement for full closed-loop run
- [x] **4.5** Display closed-loop trajectory on track plot (different color)
- [x] **4.6** Add closed-loop state/control plots (overlay with open-loop)
- [x] **4.7** Display closed-loop computation time in UI

---

## Phase 5: Parameter Tuning Interface ✓ COMPLETE
> Create sliders/inputs for all tunable parameters

### Cost Parameters (CostParams)
- [x] **5.1** Add slider/input for `v_ref` (reference velocity)
- [x] **5.2** Add sliders for state weights: `q_s`, `q_n`, `q_psi`, `q_v`
- [x] **5.3** Add sliders for control weights: `r_delta`, `r_delta_dot`, `r_tau`
- [x] **5.4** Add sliders for terminal weights: `q_s_f`, `q_n_f`, `q_psi_f`, `q_v_f`

### Constraint Parameters (ConstraintsParams)
- [x] **5.5** Add slider for `v_max` (max velocity)
- [x] **5.6** Add slider for `delta_max` (max steering angle)
- [x] **5.7** Add slider for `delta_dot_max` (max steering rate)
- [x] **5.8** Add slider for `tau_max` (max torque)
- [x] **5.9** Add slider for `car_width`

### Parameter Groups
- [x] **5.10** Organize parameters into collapsible sections (Cost, Constraints)
- [x] **5.11** Add reset-to-defaults button for each section

---

## Phase 6: Real-Time Recomputation
> Trigger recomputation on parameter changes

- [ ] **6.1** Implement parameter change detection
- [ ] **6.2** Recreate MPC controller when parameters change
- [ ] **6.3** Re-run open-loop solve on parameter change
- [ ] **6.4** Re-run closed-loop simulation on parameter change
- [ ] **6.5** Add debouncing/throttling for rapid slider changes
- [ ] **6.6** Add "Auto-recompute" toggle (vs manual "Recompute" button)
> Trigger recomputation on parameter changes

- [ ] **6.1** Implement parameter change detection
- [ ] **6.2** Recreate MPC controller when parameters change
- [ ] **6.3** Re-run open-loop solve on parameter change
- [ ] **6.4** Re-run closed-loop simulation on parameter change
- [ ] **6.5** Add debouncing/throttling for rapid slider changes
- [ ] **6.6** Add "Auto-recompute" toggle (vs manual "Recompute" button)

---

## Phase 7: Polish & UX
> Final UI improvements and stability

- [ ] **7.1** Add error handling and display for MPC failures
- [ ] **7.2** Add plot legends and axis labels
- [ ] **7.3** Add zoom/pan controls for plots
- [ ] **7.4** Add trajectory comparison mode (overlay old vs new tuning)
- [ ] **7.5** Add parameter save/load functionality (JSON/YAML)
- [ ] **7.6** Add keyboard shortcuts for common actions
- [ ] **7.7** Final testing and bug fixes

---

## File Structure

```
src/brains2/
├── src/
│   └── tools/
│       ├── CMakeLists.txt          # Build config for tuning tool
│       ├── mpc_tuning_gui.cpp      # Main application
│       ├── mpc_wrapper.hpp         # MPC controller wrapper
│       ├── mpc_wrapper.cpp
│       ├── sim_wrapper.hpp         # Simulator wrapper
│       ├── sim_wrapper.cpp
│       └── gui_state.hpp           # GUI state management
└── CMakeLists.txt                  # Updated to include tools subdirectory
```

---

## Default Parameter Values (from tests)

```cpp
// ModelParams (fixed)
ModelParams {
    .dt = 0.05,
    .m = 230.0,
    .l_R = 0.7853,
    .l_F = 0.7853,
    .C_m0 = 4.950,
    .C_r0 = 350.0,
    .C_r1 = 20.0,
    .C_r2 = 3.0,
    .t_delta = 0.02
}

// CostParams (tunable)
CostParams {
    .v_ref = 3.0,
    .q_s = 10.0,
    .q_n = 20.0,
    .q_psi = 50.0,
    .q_v = 20.0,
    .r_delta = 2.0,
    .r_delta_dot = 1.0,
    .r_tau = 0.0001,
    .q_s_f = 10000.0,
    .q_n_f = 20000.0,
    .q_psi_f = 50000.0,
    .q_v_f = 20000.0
}

// ConstraintsParams (tunable)
ConstraintsParams {
    .v_max = 10.0,
    .delta_max = 0.5,
    .delta_dot_max = 1.0,
    .tau_max = 200.0,
    .car_width = 1.55
}

// Horizon size (fixed)
Nf = 20
```

---

## Notes

- The MPC uses Frenet coordinates (s, n, psi, v, delta) internally
- Track provides `frenet_to_cartesian()` for visualization
- Sim uses Cartesian coordinates (X, Y, phi, v_x, v_y, omega, delta, tau_*)
- Coordinate conversion needed between HLC state and Sim state
- imgui_bundle includes implot for plotting - use `ImPlot::PlotLine` for trajectories
