#!/usr/bin/env python3
"""
Generate modular_robot.xacro from robot_config parameters.

This ensures URDF always uses the latest robot_config values.
Run this script whenever robot_config parameters change.
"""

from robot_config import physical as phys_config
import os

# Read the original file to extract the macro definition
urdf_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
xacro_file_src = os.path.join(urdf_dir, 'urdf', 'modular_robot.xacro')

# Also write to install directory if it exists
install_dir = os.path.join(urdf_dir, '..', '..', 'install', 'delta_robot_description', 'share', 'delta_robot_description', 'urdf')
xacro_file_install = os.path.join(install_dir, 'modular_robot.xacro') if os.path.exists(install_dir) else None

xacro_file = xacro_file_src

with open(xacro_file, 'r') as f:
    lines = f.readlines()

# Find the macro definition (between "<!-- Robot Segment Macro -->" and "<!-- Create")
macro_start_idx = None
macro_end_idx = None
for i, line in enumerate(lines):
    if '<!-- Robot Segment Macro -->' in line:
        macro_start_idx = i
    if '<!-- Create' in line and 'segment' in line.lower():
        macro_end_idx = i
        break

if macro_start_idx is None or macro_end_idx is None:
    print("ERROR: Could not find macro boundaries")
    exit(1)

# Extract macro definition
macro_definition = ''.join(lines[macro_start_idx:macro_end_idx])

# Generate xacro content
xacro_content = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="modular_serial_robot">

  <!-- Robot Dimensional Constants -->
  <!-- AUTO-GENERATED from robot_config.physical - DO NOT EDIT MANUALLY -->
  <!-- To update: run scripts/generate_urdf.py -->
  <xacro:property name="base_height" value="{phys_config.BASE_HEIGHT}"/>           <!-- {phys_config.BASE_HEIGHT*1000:.1f}mm -->
  <xacro:property name="segment_offset" value="{phys_config.SEGMENT_OFFSET}"/>        <!-- {phys_config.SEGMENT_OFFSET*1000:.1f}mm -->
  <xacro:property name="prismatic_range" value="{phys_config.PRISMATIC_RANGE}"/>       <!-- ±{phys_config.PRISMATIC_RANGE*1000:.1f}mm -->
  <xacro:property name="revolute_limit" value="{phys_config.REVOLUTE_LIMIT}"/>        <!-- ±{phys_config.REVOLUTE_LIMIT_DEG:.0f}° in radians -->
  <xacro:property name="actuator_radius" value="{phys_config.ACTUATOR_RADIUS}"/>      <!-- {phys_config.ACTUATOR_RADIUS*1000:.2f}mm - radius of actuator triangle -->

{macro_definition}
  <!-- Create a {phys_config.NUM_SEGMENTS}-segment robot -->
  <!-- AUTO-GENERATED based on NUM_SEGMENTS={phys_config.NUM_SEGMENTS} -->

"""

# Generate segment instantiations
for seg_num in range(1, phys_config.NUM_SEGMENTS + 1):
    if seg_num == 1:
        parent = "base_link"
    else:
        parent = f"seg{seg_num-1}_end_link"

    xacro_content += f"""  <!-- Segment {seg_num}: Attached to {parent} -->
  <xacro:robot_segment segment_name="seg{seg_num}" parent_link="{parent}">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robot_segment>

"""

xacro_content += "</robot>"

# Write the generated file to source directory
with open(xacro_file_src, 'w') as f:
    f.write(xacro_content)

# Also write to install directory if it exists
if xacro_file_install and os.path.exists(os.path.dirname(xacro_file_install)):
    with open(xacro_file_install, 'w') as f:
        f.write(xacro_content)
    print(f"✓ Generated {xacro_file_src}")
    print(f"✓ Generated {xacro_file_install}")
else:
    print(f"✓ Generated {xacro_file_src}")

print(f"  NUM_SEGMENTS: {phys_config.NUM_SEGMENTS}")
print(f"  BASE_HEIGHT: {phys_config.BASE_HEIGHT} m ({phys_config.BASE_HEIGHT*1000:.1f} mm)")
print(f"  SEGMENT_OFFSET: {phys_config.SEGMENT_OFFSET} m ({phys_config.SEGMENT_OFFSET*1000:.1f} mm)")
print(f"  PRISMATIC_RANGE: {phys_config.PRISMATIC_RANGE} m (±{phys_config.PRISMATIC_RANGE*1000:.1f} mm)")
print(f"  REVOLUTE_LIMIT: {phys_config.REVOLUTE_LIMIT} rad (±{phys_config.REVOLUTE_LIMIT_DEG:.0f}°)")
print(f"  ACTUATOR_RADIUS: {phys_config.ACTUATOR_RADIUS} m ({phys_config.ACTUATOR_RADIUS*1000:.2f} mm)")
