# MPC Tuning GUI

An interactive GUI tool for tuning and visualizing the High-Level Controller (HLC) Model Predictive Control (MPC) parameters. This tool allows real-time parameter adjustment and comparison between the MPC's predicted trajectory and a high-fidelity vehicle dynamics simulation.

## Overview

The MPC Tuning GUI provides:
- **Track Selection**: Choose from pre-configured tracks (alpha, beta, gamma)
- **Initial State Configuration**: Set the starting position and velocity in Frenet coordinates
- **Cost Parameter Tuning**: Adjust all MPC cost function weights
- **Constraint Parameter Tuning**: Modify velocity, steering, and torque limits
- **Open-Loop MPC Visualization**: View the MPC's predicted trajectory
- **Closed-Loop Simulation Comparison**: Compare MPC predictions against full vehicle dynamics simulation
- **Real-Time State Plots**: Visualize all state variables and controls over the prediction horizon

## Building and Running

### Build

```bash
colcon build --packages-select brains2
```

### Run

The GUI requires the acados library path to be set:

```bash
# macOS
DYLD_LIBRARY_PATH=$PWD/install/acados/lib ./build/brains2/mpc_tuning_gui

# Linux
LD_LIBRARY_PATH=$PWD/install/acados/lib ./build/brains2/mpc_tuning_gui
```

## Interface Layout

The GUI is divided into several sections:

### Controls Panel (Left Side)

#### Track Selection
- Dropdown menu to select from available tracks (alpha, beta, gamma)
- Displays track length and MPC horizon size (20 steps)
- Automatically loads track centerline, boundaries, and curvature data

#### Execution Controls
- **Auto**: Toggle automatic recomputation when parameters change (with 300ms debounce)
- **Sim**: Toggle whether to run the closed-loop simulation alongside MPC
- **Run**: Manually trigger MPC solve and simulation
- **Timing Display**: Shows solve time for HLC (MPC) and Sim in milliseconds

#### Initial State
Configure the vehicle's starting state in Frenet coordinates:
- **s**: Arc-length position along track centerline (m)
- **n**: Lateral deviation from centerline (m), range: [-2.0, 2.0]
- **ψ (psi)**: Heading angle relative to track tangent (rad), range: [-π, π]
- **v**: Longitudinal velocity (m/s), range: [0.0, 10.0]

#### Cost Parameters
Tune the MPC objective function weights:

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `v_ref` | Reference velocity (m/s) | 3.0 | [0.5, 10.0] |
| `q_s` | Progress tracking weight | 10.0 | [0, 200] |
| `q_n` | Lateral deviation weight | 20.0 | [0, 200] |
| `q_psi` | Heading error weight | 50.0 | [0, 200] |
| `q_v` | Velocity tracking weight | 20.0 | [0, 200] |
| `r_delta` | Steering angle weight | 2.0 | [0, 20] |
| `r_delta_dot` | Steering rate weight | 1.0 | [0, 20] |
| `r_tau` | Torque weight | 0.0001 | [0, 0.01] |

Terminal cost weights (applied at final horizon step):
- `q_s_f` = 10000.0
- `q_n_f` = 20000.0
- `q_psi_f` = 50000.0
- `q_v_f` = 20000.0

**Reset Cost**: Button to restore all cost parameters to defaults.

#### Constraint Parameters
Configure MPC hard constraints:

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `v_max` | Maximum velocity (m/s) | 10.0 | [1.0, 20.0] |
| `delta_max` | Maximum steering angle (rad) | 0.5 | [0.1, 1.0] |
| `delta_dot_max` | Maximum steering rate (rad/s) | 1.0 | [0.1, 5.0] |
| `tau_max` | Maximum torque (Nm) | 200.0 | [10.0, 500.0] |
| `car_width` | Vehicle width for track constraints (m) | 1.55 | - |

**Reset Constraints**: Button to restore all constraint parameters to defaults.

### Track View (Top Right)

A 2D plot showing:
- **Centerline**: Track center line (gray/blue)
- **Left/Right Boundaries**: Track edges computed from centerline + width
- **HLC Trajectory**: MPC's predicted trajectory (orange)
- **Sim Trajectory**: Simulated trajectory using full dynamics (green)
- **Start Marker**: Current initial position

The plot automatically zooms to fit the predicted trajectories with appropriate padding.

### State Plots (Bottom Right)

Six time-series plots arranged in a 2×3 grid, showing both HLC (MPC prediction) and Sim (dynamics simulation) trajectories:

| Row 1 | | |
|-------|-------|-------|
| **s** - Arc-length position (m) | **n** - Lateral deviation (m) | **ψ** - Heading angle (rad) |

| Row 2 | | |
|-------|-------|-------|
| **v** - Velocity (m/s) | **δ** - Steering angle (rad) | **T** - Torque (Nm) |

All plots share the same time axis (0 to horizon × dt = 1.0s for 20 steps at dt=0.05s).

## How It Works

### MPC (HLC)
The High-Level Controller solves an optimal control problem in Frenet coordinates using:
- **State**: (Δs, n, ψ, v, δ) - progress, lateral deviation, heading, velocity, steering angle
- **Control**: (u_δ, τ) - steering rate, rear wheel torque
- **Solver**: fatrop (interior point method)
- **Horizon**: 20 steps at 50ms = 1.0s lookahead

### Simulation
When enabled, the simulation:
1. Takes the MPC's planned control sequence
2. Applies it to a high-fidelity 4-wheel vehicle dynamics model (via acados integrator)
3. Converts the Cartesian simulation results back to Frenet coordinates
4. Displays both trajectories for comparison

This comparison helps identify:
- Model mismatch between MPC's simplified model and full dynamics
- Constraint violations in the full model
- Parameter tuning issues

## Typical Workflow

1. **Select a track** from the dropdown
2. **Set initial conditions** (position, heading, velocity)
3. **Enable Auto** for real-time updates as you tune
4. **Adjust cost parameters** and observe:
   - How trajectory changes in Track View
   - How states evolve in time plots
   - Difference between HLC prediction and Sim trajectory
5. **Fine-tune constraints** to match vehicle capabilities
6. **Compare HLC vs Sim** to validate model accuracy

## Tips

- **Large q_n values** keep the vehicle closer to centerline
- **Large q_psi values** minimize heading oscillations
- **Large r_delta_dot values** produce smoother steering
- **Sim divergence from HLC** indicates model mismatch - consider adjusting model parameters
- **Watch solve times** - should be < 10ms for real-time feasibility

## Technical Details

### Dependencies
- SDL2 + OpenGL 3.2: Window and rendering
- Dear ImGui: GUI widgets
- ImPlot: 2D plotting
- acados: MPC solver backend
- brains2 common/control/sim libraries

### Model Parameters (Fixed)
These are the internal model parameters used by the MPC:

| Parameter | Value | Description |
|-----------|-------|-------------|
| dt | 0.05 s | MPC timestep |
| m | 230 kg | Vehicle mass |
| l_R | 0.7853 m | Rear axle to CG distance |
| l_F | 0.7853 m | Front axle to CG distance |
| C_m0 | 4.950 | Motor coefficient |
| C_r0 | 350.0 | Rolling resistance |
| C_r1 | 20.0 | Linear drag |
| C_r2 | 3.0 | Quadratic drag |
| t_delta | 0.02 s | Steering actuator time constant |

### Files
- `mpc_tuning_gui.cpp`: Main application and UI rendering
- `gui_state.hpp/cpp`: Track loading and GUI state management
- `mpc_wrapper.hpp/cpp`: MPC solver interface
- `sim_wrapper.hpp/cpp`: Vehicle dynamics simulation interface
