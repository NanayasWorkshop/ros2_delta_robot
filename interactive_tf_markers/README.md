# Interactive TF Markers

This package demonstrates how to create interactive markers in RViz that broadcast TF transforms, allowing other nodes to access their positions.

## What This Does

- Creates two interactive markers in RViz: **Target** (red) and **Direction** (blue)
- Each marker can be moved in 6DOF (3 translation axes + 3 rotation axes)
- Automatically broadcasts TF transforms when markers move
- Other nodes can access marker positions via TF lookups (no custom subscribers needed!)

## Building

```bash
cd ~/ROS2
colcon build --packages-select interactive_tf_markers
source install/setup.bash
```

## Running

### Option 1: Launch Everything Including RViz (Recommended)
```bash
source ~/ROS2/install/setup.bash
ros2 launch interactive_tf_markers interactive_markers_with_rviz.launch.py
```

This starts:
- Static TF publisher (map → world)
- Interactive marker server
- RViz2

### Option 2: Launch Without RViz (Manual Setup)

**Terminal 1: Launch nodes only**
```bash
source ~/ROS2/install/setup.bash
ros2 launch interactive_tf_markers interactive_markers_no_rviz.launch.py
```

**Terminal 2: Launch RViz separately**
```bash
source ~/ROS2/install/setup.bash
rviz2
```

### Optional: Run the Example TF Listener
In a separate terminal:
```bash
source ~/ROS2/install/setup.bash
ros2 run interactive_tf_markers tf_listener_example
```

This example node shows how other nodes can access the marker positions through TF lookups.

## RViz Configuration

In RViz, you need to:

1. **Set Fixed Frame**: Change to `world` (in Global Options)

2. **Add InteractiveMarkers display**:
   - Click "Add" button
   - Select "InteractiveMarkers"
   - Set topic to: `/interactive_markers/update`

3. **Add TF display** (optional, to see the frames):
   - Click "Add" button
   - Select "TF"

4. **Enable Interact Mode**:
   - Click the "Interact" button in the toolbar (or press `i`)
   - You should now see control arrows and rings around the markers

## Using Interactive Markers

Once in "Interact" mode:

- **Arrows**: Click and drag to move along that axis (X=red, Y=green, Z=blue)
- **Rings**: Click and drag to rotate around that axis
- **Center sphere**: Click and drag to move freely in 3D

Watch Terminal 1 to see the position updates as you move the markers!

## How Other Nodes Access Positions

Other nodes can access the Target and Direction positions using TF lookups:

```python
from tf2_ros import Buffer, TransformListener

# In your node's __init__:
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# Later, when you need the position:
target_transform = self.tf_buffer.lookup_transform(
    'world',    # target frame
    'target',   # source frame (marker name)
    rclpy.time.Time()
)

# Access position:
x = target_transform.transform.translation.x
y = target_transform.transform.translation.y
z = target_transform.transform.translation.z
```

**No custom topics or subscribers needed!** The TF system handles everything.

## Available Frames

- `world` - The base reference frame
- `target` - Position of the Target marker (red sphere)
- `direction` - Position of the Direction marker (blue sphere)

## Useful Commands

View TF tree:
```bash
ros2 run tf2_tools view_frames
```

Echo a specific transform:
```bash
ros2 run tf2_ros tf2_echo world target
```

List all topics:
```bash
ros2 topic list
```

See TF broadcasts:
```bash
ros2 topic echo /tf
```

## Architecture

```
Interactive Marker Server
    ↓
User moves marker in RViz
    ↓
Feedback callback triggered
    ↓
TF Broadcaster publishes to /tf
    ↓
Other nodes use TF Listener to query positions
```

This is the **proper ROS2 way** to share spatial data between nodes!
