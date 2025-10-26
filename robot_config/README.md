# robot_config

Centralized configuration package for the delta robot system.

## Overview

This package contains all system parameters organized into logical modules, replacing the old `robot_constants.py` file with better organization and documentation.

## Structure

```
robot_config/
├── physical.py       # Robot dimensions, limits, mass properties
├── motion.py         # Ruckig, FABRIK, motion planning parameters
├── visualization.py  # RViz markers, colors, sizes, transparency
└── system.py         # Frame names, topic names, queue sizes
```

## Usage

### Import entire modules:

```python
from robot_config import physical, motion, visualization, system

num_segments = physical.NUM_SEGMENTS
tolerance = motion.FABRIK_TOLERANCE
target_color = visualization.MARKER_TARGET_COLOR
topic = system.TOPIC_MOTOR_COMMANDS
```

### Import specific values:

```python
from robot_config.physical import NUM_SEGMENTS, ACTUATOR_RADIUS
from robot_config.motion import FABRIK_TOLERANCE, RUCKIG_DELTA_TIME
from robot_config.visualization import MARKER_TARGET_COLOR
from robot_config.system import TOPIC_MOTOR_COMMANDS
```

## Migration from robot_constants.py

Old:
```python
import sys
sys.path.append('/home/yuuki/ROS2')  # ❌ Hard-coded path
import robot_constants as rc

num_motors = rc.NUM_MOTORS
```

New:
```python
from robot_config.physical import NUM_MOTORS  # ✅ Proper package import

# Or
from robot_config import physical
num_motors = physical.NUM_MOTORS
```

## Parameters by Module

### physical.py
- Segment dimensions (BASE_HEIGHT, SEGMENT_OFFSET, etc.)
- Joint limits (PRISMATIC_RANGE, REVOLUTE_LIMIT)
- Motor base angles (MOTOR_A_ANGLE, MOTOR_B_ANGLE, MOTOR_C_ANGLE)
- Robot configuration (NUM_SEGMENTS, NUM_MOTORS)
- Mass and inertia properties

### motion.py
- Ruckig motion planning (velocity, acceleration, jerk limits)
- FABRIK IK solver parameters (tolerance, max iterations)
- Motor-to-joint conversion parameters

### visualization.py
- Interactive marker properties (positions, colors, sizes)
- FABRIK visualization (S-points, J-points, chain lines)
- URDF visual properties (colors, mesh scales)

### system.py
- TF frame names (FRAME_WORLD, FRAME_TARGET, etc.)
- Topic names (TOPIC_MOTOR_COMMANDS, TOPIC_JOINT_STATES, etc.)
- Queue sizes
- Static transforms

## Benefits

- ✅ **Single source of truth** - all parameters in one package
- ✅ **No hard-coded paths** - proper Python package
- ✅ **Better organization** - logical grouping
- ✅ **Documentation** - every parameter has docstring
- ✅ **Type safety** - clear parameter types
- ✅ **Easy tuning** - change one place, affects entire system

## Version

1.0.0 - Initial release with all parameters from robot_constants.py
