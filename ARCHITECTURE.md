# ROS2 Interactive Markers & Trajectory Tracker - Architecture

## Overview
Interactive markers in RViz broadcast TF transforms. Trajectory tracker samples and records marker positions only when they change.

## System Flow

```mermaid
graph LR
    USER[User Drags Marker] --> RVIZ[RViz]
    RVIZ --> IM[interactive_marker_tf_node]
    IM -->|10Hz Timer| TF[TF Broadcaster]
    TF --> TRACKER[trajectory_tracker]
    TF --> RVIZ
    TRACKER -->|5Hz Sample| TOPIC[/trajectory topic]
```

## Packages

### interactive_tf_markers
Interactive markers that broadcast TF transforms continuously.

**Nodes:**
- `interactive_marker_tf_node`: Creates 2 markers, broadcasts TF at 10Hz
- `static_transform_publisher`: Publishes map→world transform

**Launch Files:**
- `interactive_markers.launch.py` - markers only
- `interactive_markers_with_rviz.launch.py` - markers + RViz

### trajectory_tracker
Records TF positions at 5Hz, only when positions change.

**Node:**
- `trajectory_tracker_node`: Samples TF, publishes trajectory data

**Launch Files:**
- `trajectory_tracker.launch.py` - tracker only
- `trajectory_tracker_with_rviz.launch.py` - same (for consistency)

**Messages:**
- `TrajectoryPoint.msg` - timestamp, frame_name, pose
- `Trajectory.msg` - header, points[], total_count

**Topics:**
- `/trajectory` - full trajectory data
- `/trajectory_status` - point count (Int32)
- `/clear_trajectory` - clear all points (Empty)
- `/clear_oldest` - remove N oldest points (Int32)

## Markers

| Name | Color | Size | Initial Position | TF Frame |
|------|-------|------|------------------|----------|
| **target** | Yellow | 60mm | 150, 0, 600mm | `target` |
| **direction** | Cyan | 40mm | 0, 0, 600mm | `direction` |

## TF Tree

```
map (static)
 └─ world
     ├─ target (dynamic, 10Hz)
     └─ direction (dynamic, 10Hz)
```

## Key Implementation Details

**TF Broadcasting:**
- 10Hz timer continuously broadcasts current marker poses
- Feedback callback only stores poses (no broadcasting)
- Prevents conflicts and ensures smooth marker movement

**Trajectory Tracking:**
- 5Hz sampling checks TF positions
- Only records when position changes (>1e-6 tolerance)
- Infinite memory growth (external deletion via topics)

## Launch

```bash
# Launch everything (main entry point)
cd ~/ROS2
source install/setup.bash
ros2 launch main.launch.py

# Individual packages
ros2 launch interactive_tf_markers interactive_markers.launch.py
ros2 launch trajectory_tracker trajectory_tracker.launch.py
```

## Build

```bash
cd ~/ROS2
colcon build
source install/setup.bash
```

## Usage Example

```python
# Access TF positions in your own node
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

transform = self.tf_buffer.lookup_transform(
    'world', 'target', rclpy.time.Time()
)
```

```bash
# Monitor trajectory
ros2 topic echo /trajectory_status
ros2 topic echo /trajectory

# Clear trajectory
ros2 topic pub /clear_trajectory std_msgs/Empty "{}"
ros2 topic pub /clear_oldest std_msgs/Int32 "data: 10"
```
