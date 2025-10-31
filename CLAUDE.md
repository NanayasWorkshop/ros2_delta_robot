# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Development Commands

### Building
```bash
# Build entire workspace
cd ~/ROS2
colcon build
source install/setup.bash

# Build specific packages only
colcon build --packages-select robot_config motor_trajectory_smoother fabrik_ik_solver

# Clean build
rm -rf build install log
colcon build
```

### Running the System
```bash
# Main entry point - launches complete system with RViz
cd ~/ROS2
source install/setup.bash
ros2 launch main.launch.py

# Launch individual packages for testing
ros2 launch interactive_tf_markers interactive_markers_with_rviz.launch.py
ros2 launch fabrik_ik_solver fabrik_ik_solver.launch.py
ros2 launch motor_trajectory_smoother motor_trajectory_smoother.launch.py
```

### Monitoring and Debugging
```bash
# Monitor trajectory flow
ros2 topic echo /trajectory_status        # Point count
ros2 topic echo /motor_commands           # Raw FABRIK output
ros2 topic echo /smoothed_motor_commands  # Smoothed Ruckig output

# Monitor Ruckig smoothing state (for PlotJuggler)
ros2 topic echo /ruckig/current_position
ros2 topic echo /ruckig/current_velocity
ros2 topic echo /ruckig/buffer_size

# View TF tree
ros2 run tf2_tools view_frames

# List active nodes
ros2 node list

# Clear trajectory buffer
ros2 topic pub /clear_trajectory std_msgs/Empty "{}"

# ROS PlotJuggler
ros2 run plotjuggler plotjuggler

```

## High-Level Architecture

### System Overview
This is a **delta robot inverse kinematics system** with real-time URDF visualization. The data flows through 6 ROS2 packages in sequence:

1. **Interactive Markers** → User drags markers in RViz to set target position/orientation
2. **Trajectory Tracker** → Records TF positions when changed (5Hz sampling)
3. **FABRIK IK Solver** → Solves inverse kinematics for 8-segment delta robot (24 motors)
4. **Motor Trajectory Smoother** → Applies Ruckig jerk-limited motion planning (100Hz)
5. **Motor-to-Joint Converter** → Converts motor positions to joint angles
6. **URDF Visualization** → Displays robot model in RViz via robot_state_publisher

See `ARCHITECTURE.md` for complete data flow diagram and detailed package descriptions.

### Centralized Configuration: robot_config Package

**CRITICAL**: All system parameters are centralized in the `robot_config` package. This is the single source of truth.

**Import pattern** (use this, NOT old robot_constants.py):
```python
from robot_config.physical import NUM_SEGMENTS, ACTUATOR_RADIUS, MOTOR_A_ANGLE
from robot_config.motion import FABRIK_TOLERANCE, MOTOR_MAX_VELOCITY
from robot_config.visualization import MARKER_TARGET_COLOR
from robot_config.system import TOPIC_MOTOR_COMMANDS, FRAME_TARGET
```

**Parameter modules**:
- `physical.py` - Robot dimensions, joint limits, motor base angles (90°, 330°, 210°), mass properties
- `motion.py` - Ruckig motion limits, FABRIK IK parameters, convergence tolerance
- `visualization.py` - Marker colors/sizes, FABRIK S/J point visualization
- `system.py` - TF frame names, topic names, queue sizes

**When modifying parameters**: Change them in `robot_config/robot_config/*.py`, then rebuild:
```bash
colcon build --packages-select robot_config
source install/setup.bash
# Dependent packages automatically pick up new values
```

### Key Technical Patterns

#### FABRIK IK Solver Hot Start Optimization
The FABRIK solver uses **hot start** to dramatically reduce iterations (50 → 5-7):
- Previous robot configuration (S/J points) stored in solver instance
- New IK solve uses last known position as starting point
- First solve is cold start, subsequent solves are hot starts
- Located in `fabrik_ik_solver/fabrik/` module

**Approach Point Handling**: Direction marker controls end-effector orientation. When approach vector changes, adds 0.1mm offset to target to force FABRIK iteration even when target position unchanged.

#### Ruckig Motion Planning with Look-Ahead
Motor trajectory smoother uses **intermediate waypoints** for look-ahead planning:
- Buffer queues incoming motor commands from FABRIK
- Ruckig plans through up to 10 waypoints ahead (`RUCKIG_MAX_LOOKAHEAD_WAYPOINTS`)
- Prevents overshooting on dense waypoint sequences
- Time-synchronized motion (all 24 motors arrive together)
- **List assignment requirement**: All Ruckig parameters must use whole-list assignment `input.max_velocity = [v] * DOFs` (Python bindings requirement)

Motion limits enforced:
- Max velocity: 2 mm/s
- Max acceleration: 12 mm/s²
- Max jerk: 32 mm/s³
- Update rate: 100Hz (RUCKIG_DELTA_TIME = 0.01s)

#### Motor-to-Joint Conversion Geometry
Each segment has 3 motors in delta configuration → 3 joints (2 revolute + 1 prismatic):
1. **Motor base positions** (XY plane, Z=0): A@90°, B@330°, C@210° - must match IK solver exactly
2. **Plane normal calculation**: From 3D motor positions, compute (AC × AB) normalized
3. **Joint angles from normal**: roll = -atan2(n_y, n_z), pitch = atan2(n_x, n_z)
4. **Prismatic extension**: Calculate Fermat point (geometric center) → prismatic = 2.0 × fermat_z

**Critical**: Cross product order `(AC × AB)` matters for correct upward normal direction.

#### Multithreading for Responsive Operation
FABRIK IK solver uses **MultiThreadedExecutor** pattern:
- 4 worker threads with `ReentrantCallbackGroup`
- Thread-safe locking prevents race conditions on processing flags
- IK solving doesn't block marker dragging or TF broadcasting
- Professional ROS2 pattern for non-blocking operation

### URDF Auto-Generation System

The robot URDF is **automatically generated** from `robot_config` parameters at build time.

**Generation flow**:
1. `delta_robot_description/scripts/generate_urdf.py` reads from `robot_config`
2. Writes `delta_robot_description/urdf/modular_robot.urdf.xacro`
3. CMake configures URDF at build time
4. Launch file loads URDF via robot_state_publisher

**To modify robot structure**:
1. Edit parameters in `robot_config/robot_config/physical.py` (e.g., NUM_SEGMENTS, BASE_HEIGHT)
2. Rebuild: `colcon build --packages-select delta_robot_description robot_config`
3. URDF regenerates automatically with new parameters

**URDF structure** (per segment):
- `segN_base_link` → `segN_joint1` (revolute X) → `segN_joint2` (revolute Y) → `segN_joint3` (prismatic Z)
- Mimic joints (4-6) mirror revolute joints (1-3) for visual symmetry
- Static TF: world→base_link published by static_transform_publisher

### External Dependencies

#### Ruckig Library
Located in `external_libs/ruckig/` as ROS2 package. Built with colcon.
- **Purpose**: Jerk-limited trajectory generation (Type V generator)
- **Python bindings**: Used via `from ruckig import Ruckig, InputParameter, OutputParameter`
- **No separate installation needed** - included in workspace

Key reference: [Ruckig documentation](https://ruckig.com) for advanced features.

## Common Development Patterns

### Adding Parameters
1. Add to appropriate `robot_config/robot_config/*.py` module with docstring
2. Import in target node: `from robot_config.module import PARAMETER_NAME`
3. Rebuild `robot_config` package
4. Never hard-code values - always use centralized parameters

### Working with FABRIK Algorithm
- Algorithm implementation in `fabrik_ik_solver/fabrik/` directory (installed as Python package)
- Node implementation in `fabrik_ik_solver/fabrik_ik_solver/fabrik_ik_solver_node.py`
- Uses cone constraints (60° half-angle = 2 × REVOLUTE_LIMIT)
- Returns 24 motor positions (3 motors × 8 segments) + convergence info

### Debugging Motion Planning Issues
1. Check Ruckig buffer size: `ros2 topic echo /ruckig/buffer_size`
2. Compare raw vs smoothed: `/fabrik/motor_positions` vs `/ruckig/current_position`
3. Verify motion limits not violated (check acceleration/jerk plots in PlotJuggler)
4. Look for velocity/acceleration discontinuities indicating constraint violations

### TF Frame Conventions
- All frames defined in `robot_config/system.py`
- Static tree: map → world → base_link → seg1_base_link → ... → seg8_end_link
- Dynamic frames: target, direction (updated at 10Hz by interactive markers)
- Reference frame for IK: 'world'

## Package Structure

**Core packages** (in dependency order):
- `robot_config` - Centralized parameters (must build first)
- `interactive_tf_markers` - User interaction via RViz markers
- `trajectory_tracker` - TF position recording (custom Trajectory.msg)
- `fabrik_ik_solver` - IK solving (custom MotorCommand.msg, FabrikVisualization.msg)
- `motor_trajectory_smoother` - Ruckig smoothing (uses fabrik_ik_solver/msg)
- `motor_to_joint_converter` - Geometry conversion (publishes sensor_msgs/JointState)
- `delta_robot_description` - URDF and visualization

**Supporting**:
- `external_libs/ruckig/` - Motion planning library (ROS2 wrapper)
- `main.launch.py` - System entry point at workspace root
- `rviz/main.rviz` - Complete system RViz config

## Important Notes

### Package Install Pattern
All Python nodes use `ament_python` packages with proper package structure:
- Module installed via `setup.py` with `packages=['package_name']`
- Nodes located in `package_name/package_name/node_name.py`
- Executables configured via `setup.py` entry_points
- Import within workspace: `from package_name.module import Class`

### Message Dependencies
- `trajectory_tracker_interfaces` - Trajectory.msg, TrajectoryPoint.msg
- `fabrik_ik_solver` - MotorCommand.msg, FabrikVisualization.msg
- Other packages use std_msgs, sensor_msgs, geometry_msgs

### Workspace Layout
- Source packages in root: `fabrik_ik_solver/`, `motor_trajectory_smoother/`, etc.
- Build artifacts in `build/`, `install/`, `log/` (git-ignored)
- Launch file at root: `main.launch.py` (includes all package launch files)
- RViz config at root: `rviz/main.rviz`

### Migration History
Recent commits show migration from monolithic `robot_constants.py` to `robot_config` package:
- Old: `sys.path.append('/home/yuuki/ROS2'); import robot_constants`
- New: `from robot_config.physical import PARAM`
- Benefits: No hard-coded paths, proper Python package, single source of truth
