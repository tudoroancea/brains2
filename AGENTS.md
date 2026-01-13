# brains2 Development Guide

## Project Overview

`brains2` is a ROS2 autonomous vehicle control stack focused on model predictive control (MPC) for racing applications. The project simulates a 4-wheel vehicle using complex dynamics models and controls it using NMPC in curvilinear (Frenet) coordinates.

## Setup and Installation

### Prerequisites

1. **Miniforge3**: Install miniforge3 for cross-platform conda support:
   ```bash
   curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
   bash Miniforge3-$(uname)-$(uname -m).sh
   ```

2. **Clone with submodules** (acados is the only submodule):
   ```bash
   git clone https://github.com/tudoroancea/brains2.git --recursive
   cd brains2
   ```

### Environment Setup

The project uses conda to manage almost all dependencies for cross-platform compatibility (Linux and macOS):

```bash
# Install conda-lock in base environment
conda install -n base conda-lock

# Create and activate the brains2 environment from lock file
conda activate base
./scripts/env_sync.sh
conda deactivate
conda activate brains2
```

### Building

```bash
./scripts/build.sh
. install/setup.sh
pip3 install -e $ACADOS_SOURCE_DIR/interfaces/acados_template
```

**Important**: `build.sh` modifies `install/setup.sh` to append `ACADOS_SOURCE_DIR`. This must be sourced after every build.

### Visualization (Foxglove)

Install the Foxglove extension for joystick control and service call buttons:

```bash
cd foxglove_extensions
npm install
npm run local-install
```

Open Foxglove Desktop and import `foxglove_template.json` for the pre-configured layout.

## Architecture

### Package Structure

```
src/
├── acados/                    # Git submodule - optimization solver
├── brains2/
│   ├── include/               # Public headers
│   ├── src/
│   │   ├── common/            # Shared utilities (track database, splines)
│   │   ├── sim/               # Vehicle dynamics simulation
│   │   ├── control/           # MPC controllers (high/low level)
│   │   ├── estimation/        # Track estimation, spline fitting
│   │   ├── coordination/      # FSM and node coordination
│   │   ├── slam/              # Localization (fake implementation)
│   │   └── tools/             # GUI tools (MPC tuning)
│   ├── msg/                   # ROS2 message definitions
│   ├── meshes/                # 3D meshes for visualization
│   └── tests/                 # Unit tests
```

### Key Modules

1. **sim** (`src/sim/`): Implements vehicle dynamics using acados integrators
   - Kinematic bicycle model (low velocity)
   - Dynamic 4-wheel model with Pacejka tire model
   - `SimNode` publishes pose/velocity; accepts target controls

2. **control** (`src/control/`): NMPC in Frenet frame
   - State: (Δs, n, ψ, v, δ) - track progress, lateral deviation, heading, velocity, steering
   - Control: (u_δ, τ) - steering rate, torque
   - Uses casadi + fatrop solver

3. **estimation** (`src/estimation/`): Track estimation from cone detections
   - Spline-based track representation (center line, heading, curvature, width)

4. **common** (`src/common/`): Shared utilities
   - Track database management
   - Spline fitting utilities

5. **tools** (`src/tools/`): GUI tools for MPC tuning
   - `mpc_tuning_gui` - Interactive GUI for tuning MPC parameters
   - Visualizes track, open-loop and closed-loop trajectories
   - Supports real-time parameter adjustment and comparison

### ROS2 Topics

Key topics published/subscribed:
- `/brains2/pose` - vehicle pose
- `/brains2/velocity` - vehicle velocity
- `/brains2/acceleration` - vehicle acceleration
- `/brains2/target_controls` - control inputs (steering, torques)
- `/brains2/track_estimation` - estimated track spline
- `/brains2/control/debug_info` - OCP reference vs prediction comparison

## Scripts

| Script | Purpose |
|--------|---------|
| `scripts/build.sh` | Build acados and brains2 packages; appends ACADOS_SOURCE_DIR to setup.sh |
| `scripts/env_sync.sh` | Sync conda environment from conda-lock.yml |
| `scripts/env_lock.sh` | Lock environment to env.yml (updates conda-lock.yml) |
| `scripts/test.sh` | Run unit tests |
| `scripts/lint.sh` | Run code formatting and linting checks |
| `scripts/purge.sh` | Clean build artifacts |

## Adding Dependencies

1. **Preferred**: Add to `env.yml` for conda installation
2. **Fallback**: Git submodule pinned to specific commit (only acados currently)
3. **After changes**: Run `./scripts/env_lock.sh` then `./scripts/env_sync.sh`

## Foxglove Extension

Custom panels in `foxglove_extensions/`:
- **Joystick Controller**: Publishes to `/brains2/target_controls` for manual control
- **Call Service Button**: Calls ROS2 services (reset, start, publish_cones)

## Testing

```bash
./scripts/test.sh
```

Unit tests exist for:
- `test_track_database` - common module
- `test_track` - track representation
- `test_spline_fittings` - estimation module
- `test_high_level_controller` - control module

## Code Style

### C++ Conventions

The project uses C++17 with `.clang-format` based on Google style with these modifications:
- Column limit: 100 characters
- Access modifier offset: -4 (private/protected indented less)
- Pointer alignment: Right
- Braces inserted for all control statements

#### Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Classes | PascalCase | `SimNode`, `HighLevelController` |
| Structs | PascalCase | `Sim::State`, `Sim::Control` |
| Functions | snake_case | `rpy_to_quaternion()`, `clip()` |
| Variables | snake_case | `pose_msg`, `vel_sub` |
| Class members | trailing underscore `_` | `pose_pub_`, `control_` |
| Static variables | snake_case | `last_lap_time` |
| Constants | UPPER_SNAKE_CASE | `MAX_VELOCITY`, `DIM = 10` |
| Namespaces | snake_case | `brains2::sim`, `brains2::common` |
| ROS2 nodes | snake_case | `sim_node`, `control_node` |
| Topics | kebab-case | `/brains2/pose`, `/brains2/target_controls` |
| Services | kebab-case | `/brains2/reset`, `/brains2/start` |

#### Header File Conventions

```cpp
// Copyright header with MIT license
#ifndef BRAINS2__MODULE__FILE_HPP_
#define BRAINS2__MODULE__FILE_HPP_

#include <module>
#include "brains2/external/..."  // internal first
#include <third_party>           // then external

namespace brains2 {
namespace module {

class MyClass {
public:
    // Doxygen-style documentation for public API
    /*
     * @brief Brief description.
     * @param param_name Description of parameter.
     * @return Description of return value.
     */
    ReturnType method(const ParamType& param);

private:
    // Private members use trailing underscore
    int member_;
    std::unique_ptr<Something> something_;
};

}  // namespace module
}  // namespace brains2

#endif  // BRAINS2__MODULE__FILE_HPP_
```

#### ROS2 Node Implementation Patterns

```cpp
class NodeName : public rclcpp::Node {
public:
    NodeName() : Node("node_name") {
        // Declare parameters in constructor
        declare_parameter("param_name", default_value);
        declare_parameter("another_param", 10.0);

        // Create publishers/subscribers
        pub_ = create_publisher<MsgType>("/topic/name", 10);
        sub_ = create_subscription<MsgType>(
            "/topic/name", 10,
            std::bind(&NodeName::callback, this, std::placeholders::_1));

        // Create services
        srv_ = create_service<SrvType>(
            "/service/name",
            [this](auto req, auto res) { /* handler */ });
    }

private:
    // Member variables with trailing underscore
    Publisher<MsgType>::SharedPtr pub_;
    rclcpp::Subscription<MsgType>::SharedPtr sub_;
    double param_;
    std::unique_ptr<Something> something_;

    // Callback methods
    void callback(const MsgType::ConstSharedPtr msg) {
        // Processing logic
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeName>();
    try {
        rclcpp::spin(node);
    } catch (std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
```

#### Error Handling

- Use `tl::expected<T, Error>` for recoverable errors (see `brains2/external/expected.hpp`)
- Use exceptions for unrecoverable errors (caught in `main()`)
- Use `RCLCPP_ERROR/DEBUG/INFO` for logging
- Use `DiagnosticArray` for publishing runtime diagnostics

#### ROS2 QoS Profiles

- Default: `rmw_qos_profile_default` (depth=10)
- Use explicit QoS when needed for latched/topics with different reliability

### Python Conventions

- Python code linted with `ruff`
- Follow PEP8 with line length matching clang-format (100 chars)
- Type hints preferred for function signatures
- Use `snake_case` for functions and variables

### Copyright Headers

All files must include the MIT copyright header (checked by `ament_copyright`):

```cpp
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
```

### Manual Code Review Checklist

These items are not checked by formatters/linters:

- [ ] **TODO comments**: Add ticket reference or `@todo` note explaining why
- [ ] **Magic numbers**: Extract to named constants or config parameters
- [ ] **Long functions**: Consider extracting to private helper methods
- [ ] **Deep nesting**: Max 3-4 levels; consider early returns
- [ ] **Member access**: Use `this->` for member variables (consistency)
- [ ] **Parameter comments**: Document units (m, rad, s, etc.) in docstrings
- [ ] **Variable names**: Ensure names are descriptive; avoid single letters except loop indices
- [ ] **Const-correctness**: Use `const` for parameters that shouldn't be modified
- [ ] **SharedPtr patterns**: Use `SharedPtr` for ROS callbacks; `unique_ptr` for owned objects
- [ ] **Include order**: System → External → Internal (checked by clang-format)
- [ ] **Namespace comments**: Use `}  // namespace name` style (checked by clang-format)

## Known Issues

- conda-lock has a bug on linux-64 requiring manual fix (see conda-lock PR #776)
- Implicit integrator used even for kinematic model due to stiff actuator dynamics

## Build Configuration

- **CMAKE_BUILD_TYPE**: Debug by default, RelWithDebInfo for production
- **Compilers**: clang/clang++ from conda environment
- **acados**: Built first, then brains2 links against it

### MPC Tuning GUI

The `mpc_tuning_gui` executable is built as part of the brains2 package. See [`src/brains2/src/tools/README.md`](src/brains2/src/tools/README.md) for full documentation.

```bash
# Build and run
colcon build --packages-select brains2

# macOS
DYLD_LIBRARY_PATH=$PWD/install/acados/lib ./build/brains2/mpc_tuning_gui

# Linux
LD_LIBRARY_PATH=$PWD/install/acados/lib ./build/brains2/mpc_tuning_gui
```

## Configuration Files

| File | Purpose |
|------|---------|
| `car_constants.yaml` | Vehicle physical parameters (mass, dimensions, tire coefficients) |
| `env.yml` | Conda environment specification (source of truth) |
| `conda-lock.yml` | Locked dependencies (generated; don't edit manually) |
| `virtual-packages.yml` | Hardware assumptions (CPU architecture) |
| `mcap_writer_options.yml` | ROS2 bag recording options |

### Config File Conventions

YAML files follow these patterns:

```yaml
# Copyright header (optional for YAML)
# All units in SI (meters, radians, seconds, kilograms)

section_name:
  param_name: value  # snake_case for keys
  # Comments for non-obvious values
  dimensional_param: 1.5706  # e.g., wheelbase in meters
```

**Units**: Always SI units unless otherwise documented:
- Length: meters (m)
- Angle: radians (rad)
- Time: seconds (s)
- Mass: kilograms (kg)
- Force: Newtons (N)
- Torque: Newton-meters (Nm)
- Velocity: meters/second (m/s)
- Inertia: kg⋅m²
