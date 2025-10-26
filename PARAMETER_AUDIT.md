# Parameter Audit Report

## Summary

Comprehensive audit of all magic numbers and parameters across 6 ROS2 packages.

**Status:**
- ✅ **GOOD:** Most robot physical parameters already centralized in `robot_constants.py`
- ❌ **CRITICAL:** Hard-coded paths in multiple files
- ⚠️ **WARNING:** Duplicate parameters between URDF and Python
- ⚠️ **WARNING:** Visualization parameters scattered across nodes

---

## Critical Issues

### 1. Hard-Coded System Paths
**Location:** Multiple packages
```python
sys.path.append('/home/yuuki/ROS2')  # ❌ Hard-coded absolute path
```

**Files affected:**
- `motor_trajectory_smoother/motor_trajectory_smoother_node.py:11`
- `motor_to_joint_converter/motor_to_joint_converter_node.py:12`
- `fabrik_ik_solver/fabrik/*.py` (multiple files)

**Problem:** Won't work on different machines or installations

---

### 2. Duplicate Parameters
**robot_constants.py vs modular_robot.xacro:**

| Parameter | robot_constants.py | modular_robot.xacro |
|-----------|-------------------|---------------------|
| BASE_HEIGHT | 0.0115 | 0.0115 |
| SEGMENT_OFFSET | 0.0615 | 0.0615 |
| PRISMATIC_RANGE | 0.0108 | 0.0108 |
| REVOLUTE_LIMIT | 0.5236 | 0.5236 |
| ACTUATOR_RADIUS | 0.02485 | 0.02485 |

**Problem:** Must manually update both files - easy to get out of sync

---

## Parameters by Category

### ROBOT PHYSICAL PARAMETERS ✅
**Already in robot_constants.py** - This is good!

```python
# Segment dimensions (meters)
BASE_HEIGHT = 0.0115              # 11.5mm
SEGMENT_OFFSET = 0.0615           # 61.5mm
PRISMATIC_RANGE = 0.0108          # ±10.8mm
REVOLUTE_LIMIT = 0.5236           # ±30° in radians
ACTUATOR_RADIUS = 0.02485         # 24.85mm

# Robot configuration
NUM_SEGMENTS = 8
JOINTS_PER_SEGMENT = 3
NUM_MOTORS = 24

# Motion planning
MOTOR_MAX_VELOCITY = 0.002        # 2 mm/s
MOTOR_MAX_ACCELERATION = 0.036    # 36 mm/s²
MOTOR_MAX_JERK = 0.128            # 128 mm/s³
RUCKIG_DELTA_TIME = 0.01          # 10ms = 100Hz
```

---

### INTERACTIVE MARKERS ⚠️
**Location:** `interactive_tf_markers/interactive_marker_tf_node.py`

**Target Marker:**
```python
position = [0.150, 0.0, 0.600]    # 150mm, 0mm, 600mm (line 29)
color = [1.0, 1.0, 0.0]           # Yellow RGB (line 29)
size = 0.06                        # 60mm diameter (line 29)
```

**Direction Marker:**
```python
position = [0.0, 0.0, 0.600]      # 0mm, 0mm, 600mm (line 30)
color = [0.0, 1.0, 1.0]           # Cyan RGB (line 30)
size = 0.04                        # 40mm diameter (line 30)
```

**Other:**
```python
tf_broadcast_rate = 0.1            # 10Hz (line 36)
control_scale = 0.15               # Control handle size (line 50)
marker_alpha = 0.8                 # Transparency (line 74)
```

---

### TRAJECTORY TRACKER ⚠️
**Location:** `trajectory_tracker/`

**Node parameters (trajectory_tracker_node.py):**
```python
position_tolerance = 1e-6          # Position change detection (line 95-97)
warning_throttle = 1.0             # Seconds between warnings (line 126)
queue_sizes = 10                   # All publishers/subscribers (lines 39,44,52,58)
```

**Launch file parameters (trajectory_tracker.launch.py):**
```python
tracked_frames = ['target', 'direction']  # Frames to track (line 13)
reference_frame = 'world'                 # Reference frame (line 14)
sample_rate = 5.0                         # Hz (line 15)
```

---

### FABRIK IK SOLVER ⚠️
**Location:** `fabrik_ik_solver/`

**Algorithm parameters:**
```python
tolerance = 0.001                  # 1mm convergence (line 28, launch:19)
max_iterations = 50                # Max FABRIK iterations (line 29, launch:20)
approach_change_threshold = 1e-6   # Detect approach change (line 157)
target_offset = 1e-4               # 0.1mm offset when approach changes (line 161)
```

**Visualization parameters (fabrik_ik_solver_node.py):**
```python
s_point_size = 0.030               # 30mm blue spheres (lines 287-289)
j_point_size = 0.030               # 30mm green spheres (lines 309-311)
chain_line_width = 0.005           # 5mm line width (lines 326, 349)
point_alpha = 0.8                  # Point transparency (lines 293, 315)
line_alpha = 0.6                   # Line transparency (lines 330, 353)
```

**Launch file flags:**
```python
num_segments = 8                   # Number of segments (line 18)
enable_visualization = True        # Show markers (line 21)
use_hot_start = True               # Use previous solution (line 22)
```

---

### MOTOR TRAJECTORY SMOOTHER ⚠️
**Location:** `motor_trajectory_smoother/`

**Node parameters:**
```python
publish_rate = 100.0               # Hz trajectory publishing (line 26, launch:15)
max_lookahead = 10                 # Waypoints for Ruckig (line 150)
initial_velocity = 0.0             # Start from rest (line 133)
initial_acceleration = 0.0         # Start from rest (line 134)
queue_sizes = 10                   # All topics (lines 54,61,65-69)
```

**Hard-coded path:**
```python
sys.path.append('/home/yuuki/ROS2')  # ❌ CRITICAL (line 11)
```

---

### MOTOR TO JOINT CONVERTER ⚠️
**Location:** `motor_to_joint_converter/motor_to_joint_converter_node.py`

**Conversion parameters:**
```python
division_epsilon = 1e-10           # Prevent division by zero (line 162)
prismatic_multiplier = 2.0         # Fermat Z to joint extension (line 115)
fermat_angle_offset = math.pi / 3  # 60° angle offset (lines 163-165)
```

**Motor base angles (matches IK solver):**
```python
motor_A_angle = 90°                # (0, r) - line 182
motor_B_angle = -math.pi / 6.0     # -30° = 330° (line 187)
motor_C_angle = -5*math.pi / 6.0   # -150° = 210° (line 193)
```

**Hard-coded path:**
```python
sys.path.append('/home/yuuki/ROS2')  # ❌ CRITICAL (line 12)
```

---

### URDF / DELTA_ROBOT_DESCRIPTION ⚠️
**Location:** `delta_robot_description/urdf/modular_robot.xacro`

**Dimensional properties (DUPLICATES robot_constants.py):**
```xml
<xacro:property name="base_height" value="0.0115"/>      <!-- 11.5mm -->
<xacro:property name="segment_offset" value="0.0615"/>   <!-- 61.5mm -->
<xacro:property name="prismatic_range" value="0.0108"/>  <!-- ±10.8mm -->
<xacro:property name="revolute_limit" value="0.5236"/>   <!-- ±30° -->
<xacro:property name="actuator_radius" value="0.02485"/> <!-- 24.85mm -->
```

**Mass properties:**
```xml
base_mass = 1.0 kg
link_mass = 0.1 kg
middle_mass = 0.5 kg
```

**Inertia values:**
```xml
base_inertia = 0.010
link_inertia = 0.001
middle_inertia = 0.005
```

**Visual properties:**
```xml
mesh_scale = [0.001, 0.001, 0.001]  # STL in mm, convert to m
base_color = rgba(0.5, 0.5, 0.5, 1.0)  # Gray
middle_color = rgba(0.0, 0.0, 1.0, 1.0)  # Blue
```

**Launch file (display.launch.py):**
```python
static_tf_args = ['0', '0', '0', '0', '0', '0', 'world', 'base_link']  # line 41
```

---

### ROS2 SYSTEM PARAMETERS ⚠️
**Scattered across all packages:**

```python
queue_size = 10  # Every publisher/subscriber in every node
```

**Frame names (should be centralized):**
```python
frames = ['world', 'map', 'base_link', 'target', 'direction']
```

---

## Recommended Solution

### Create centralized `robot_config` package

```
robot_config/
├── robot_config/
│   ├── __init__.py
│   ├── physical.py          # Robot dimensions, limits (from robot_constants.py)
│   ├── motion.py            # Ruckig, FABRIK algorithm params
│   ├── visualization.py     # Marker sizes, colors, transparency
│   ├── system.py            # Queue sizes, frame names, rates
│   └── paths.py             # Dynamic path resolution (fix hard-coded paths)
├── config/
│   └── robot_params.yaml    # Optional: override defaults
├── package.xml
├── setup.py
└── CMakeLists.txt
```

### Benefits:
- ✅ **Single source of truth** - all parameters in one place
- ✅ **No hard-coded paths** - dynamic package path resolution
- ✅ **No duplicates** - URDF reads from YAML config
- ✅ **Easy tuning** - change one file, rebuild
- ✅ **Type safety** - Python typing, validation
- ✅ **Documentation** - docstrings for each parameter
- ✅ **Version control** - proper ROS2 package

### Migration Path:
1. Create `robot_config` package
2. Move all parameters from `robot_constants.py`
3. Update all nodes to import from `robot_config`
4. Generate YAML for URDF xacro
5. Remove hard-coded paths
6. Test and commit

---

## Statistics

**Total packages audited:** 6
- interactive_tf_markers
- trajectory_tracker
- fabrik_ik_solver
- motor_trajectory_smoother
- motor_to_joint_converter
- delta_robot_description

**Total parameter categories:** 7
- Robot physical (✅ already centralized)
- System paths (❌ critical issue)
- Interactive markers (⚠️ needs centralization)
- Trajectory tracker (⚠️ needs centralization)
- FABRIK solver (⚠️ needs centralization)
- Motion smoother (⚠️ needs centralization)
- Motor converter (⚠️ needs centralization)
- URDF/Visualization (⚠️ duplicates)

**Hard-coded paths found:** 3+ files
**Duplicate parameters:** 5 (robot_constants.py ↔ URDF xacro)
**Magic numbers:** 50+ scattered across files

---

## Next Steps

1. ✅ Review this audit
2. Create `robot_config` package structure
3. Migrate parameters from `robot_constants.py`
4. Update all imports
5. Remove hard-coded paths
6. Sync URDF with config
7. Test entire system
8. Commit changes
