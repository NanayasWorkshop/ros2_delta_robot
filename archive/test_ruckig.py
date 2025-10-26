#!/usr/bin/env python3
"""
Simple Ruckig test - sine wave motion for 3 motors
Tests if pass_to_input() works correctly
"""

import numpy as np
import time
from ruckig import Ruckig, InputParameter, OutputParameter, Result

# Constants
NUM_MOTORS = 3
DELTA_TIME = 0.01  # 10ms = 100Hz
MAX_VELOCITY = 0.002
MAX_ACCELERATION = 0.012
MAX_JERK = 0.032

# Initialize Ruckig
ruckig = Ruckig(NUM_MOTORS, DELTA_TIME)
input_param = InputParameter(NUM_MOTORS)
output_param = OutputParameter(NUM_MOTORS)

# Set kinematic limits - assign whole list at once!
input_param.max_velocity = [MAX_VELOCITY] * NUM_MOTORS
input_param.max_acceleration = [MAX_ACCELERATION] * NUM_MOTORS
input_param.max_jerk = [MAX_JERK] * NUM_MOTORS

# Initialize current state (starting position) - assign whole list!
input_param.current_position = [0.0] * NUM_MOTORS
input_param.current_velocity = [0.0] * NUM_MOTORS
input_param.current_acceleration = [0.0] * NUM_MOTORS

print("=" * 60)
print("Ruckig Test - Sine Wave Motion")
print("=" * 60)
print(f"Motors: {NUM_MOTORS}")
print(f"Delta time: {DELTA_TIME}s")
print(f"Max velocity: {MAX_VELOCITY} m/s")
print(f"Max acceleration: {MAX_ACCELERATION} m/s²")
print(f"Max jerk: {MAX_JERK} m/s³")
print("=" * 60)

# Generate waypoints - sine wave
waypoints = []
for t in np.linspace(0, 2*np.pi, 20):  # 20 waypoints over one sine period
    waypoint = [
        0.1 * np.sin(t),           # Motor 0: sine wave
        0.1 * np.sin(t + np.pi/2), # Motor 1: phase shifted
        0.1 * np.sin(t + np.pi),   # Motor 2: phase shifted more
    ]
    waypoints.append(waypoint)

print(f"\nGenerated {len(waypoints)} waypoints")
print("\nProcessing waypoints...")
print("-" * 60)

# Process each waypoint
for idx, waypoint in enumerate(waypoints):
    # Set target
    for i in range(NUM_MOTORS):
        input_param.target_position[i] = waypoint[i]
        input_param.target_velocity[i] = 0.0  # Stop at each waypoint
        input_param.target_acceleration[i] = 0.0

    print(f"\nWaypoint {idx + 1}/{len(waypoints)}")
    print(f"  Target: [{waypoint[0]:.4f}, {waypoint[1]:.4f}, {waypoint[2]:.4f}]")

    # Check limits before update
    print(f"  Max velocity before update: [{input_param.max_velocity[0]:.6f}, {input_param.max_velocity[1]:.6f}, {input_param.max_velocity[2]:.6f}]")

    # Update Ruckig (one step)
    result = ruckig.update(input_param, output_param)

    if result == Result.Error or result == Result.ErrorInvalidInput:
        print(f"  ❌ ERROR: {result}")
        print(f"  Current pos: {list(input_param.current_position)}")
        print(f"  Target pos: {list(input_param.target_position)}")
        print(f"  Max vel: {list(input_param.max_velocity)}")
        break

    # Show output
    print(f"  Result: {result}")
    print(f"  New pos: [{output_param.new_position[0]:.4f}, {output_param.new_position[1]:.4f}, {output_param.new_position[2]:.4f}]")
    print(f"  New vel: [{output_param.new_velocity[0]:.6f}, {output_param.new_velocity[1]:.6f}, {output_param.new_velocity[2]:.6f}]")

    # Pass output to input for next cycle
    output_param.pass_to_input(input_param)

    # Check limits after pass_to_input
    print(f"  Max velocity after pass_to_input: [{input_param.max_velocity[0]:.6f}, {input_param.max_velocity[1]:.6f}, {input_param.max_velocity[2]:.6f}]")

    # Re-set limits (in case pass_to_input cleared them)
    for i in range(NUM_MOTORS):
        input_param.max_velocity[i] = MAX_VELOCITY
        input_param.max_acceleration[i] = MAX_ACCELERATION
        input_param.max_jerk[i] = MAX_JERK

    print(f"  Max velocity after re-setting: [{input_param.max_velocity[0]:.6f}, {input_param.max_velocity[1]:.6f}, {input_param.max_velocity[2]:.6f}]")

    # Simulate reaching the target (in real system this would run at 100Hz until Result.Finished)
    if result == Result.Finished:
        print(f"  ✓ Reached waypoint")

print("\n" + "=" * 60)
print("Test complete!")
print("=" * 60)
