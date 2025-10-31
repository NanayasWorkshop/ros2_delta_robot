#!/usr/bin/env python3
"""
Simple test: Give Ruckig ALL waypoints at once, let it follow them.
Should see smooth sine wave following with maintained velocity.
"""

import numpy as np
import matplotlib.pyplot as plt
from ruckig import Ruckig, InputParameter, OutputParameter, Result

def test_batch_waypoints():
    print("=" * 70)
    print("Testing Ruckig with Batch Sine Waypoints")
    print("=" * 70)

    # Generate sine wave waypoints
    frequency = 0.5  # Hz
    duration = 4.0   # seconds (2 full cycles)
    sample_rate = 50  # waypoints per second

    t = np.linspace(0, duration, int(duration * sample_rate))
    sine_positions = np.sin(2 * np.pi * frequency * t)

    print(f"Generated {len(sine_positions)} sine waypoints")
    print(f"Frequency: {frequency} Hz")
    print(f"Duration: {duration}s")
    print(f"Sample rate: {sample_rate} Hz")
    print()

    # Setup Ruckig
    control_cycle = 0.01  # 100 Hz
    max_waypoints = len(sine_positions) - 1  # All waypoints as intermediate

    ruckig = Ruckig(1, control_cycle, max_waypoints)
    input_param = InputParameter(1)
    output_param = OutputParameter(1, max_waypoints)

    # Limits
    input_param.max_velocity = [5.0]  # High limit to not constrain
    input_param.max_acceleration = [10.0]
    input_param.max_jerk = [50.0]

    # Current state (start at first waypoint)
    input_param.current_position = [sine_positions[0]]
    input_param.current_velocity = [0.0]
    input_param.current_acceleration = [0.0]

    # Set ALL waypoints at once!
    # intermediate_positions = all but last point
    # target_position = last point
    input_param.intermediate_positions = [[p] for p in sine_positions[1:-1]]
    input_param.target_position = [sine_positions[-1]]
    input_param.target_velocity = [0.0]  # Stop at end
    input_param.target_acceleration = [0.0]

    print(f"Feeding Ruckig:")
    print(f"  - Start position: {sine_positions[0]:.3f}")
    print(f"  - {len(input_param.intermediate_positions)} intermediate waypoints (cruise through)")
    print(f"  - Target position: {sine_positions[-1]:.3f} (stop here)")
    print()
    print("Waiting for cloud API calculation...")

    # First update triggers cloud calculation
    result = ruckig.update(input_param, output_param)

    if result == Result.Error or result == Result.ErrorInvalidInput:
        print(f"❌ ERROR: {result}")
        return

    print("✓ Trajectory calculated!")
    print(f"  Duration: {output_param.trajectory.duration:.3f}s")
    print()

    # Now run the trajectory
    actual_positions = []
    actual_velocities = []
    actual_accelerations = []
    actual_times = []

    current_time = 0.0
    step = 0

    print("Following trajectory...")
    print()

    while result == Result.Working:
        # Record state
        actual_positions.append(output_param.new_position[0])
        actual_velocities.append(output_param.new_velocity[0])
        actual_accelerations.append(output_param.new_acceleration[0])
        actual_times.append(current_time)

        # Print progress
        if step % 100 == 0:
            print(f"Step {step:4d} | Time: {current_time:5.2f}s | "
                  f"Pos: {output_param.new_position[0]:7.3f} | "
                  f"Vel: {output_param.new_velocity[0]:7.3f} | "
                  f"Acc: {output_param.new_acceleration[0]:7.3f}")

        # Pass output to input for next step
        output_param.pass_to_input(input_param)

        # Update
        result = ruckig.update(input_param, output_param)

        current_time += control_cycle
        step += 1

        # Safety limit
        if step > 10000:
            print("⚠️  Max steps reached")
            break

    print()
    print(f"✓ Trajectory finished in {step} steps ({current_time:.2f}s)")
    print()

    # Analysis
    velocities = np.array(actual_velocities)

    print("=" * 70)
    print("Analysis:")
    print("=" * 70)
    print(f"Max velocity: {np.max(np.abs(velocities)):.3f} m/s")
    print(f"Mean |velocity|: {np.mean(np.abs(velocities)):.3f} m/s")
    print(f"Min |velocity|: {np.min(np.abs(velocities)):.3f} m/s")
    print()

    # Check for stops in the middle (exclude first/last 10%)
    mid_start = int(len(velocities) * 0.1)
    mid_end = int(len(velocities) * 0.9)
    mid_vels = velocities[mid_start:mid_end]
    stops = np.sum(np.abs(mid_vels) < 0.01)

    print(f"Stops in middle 80% (|v| < 0.01 m/s): {stops}/{len(mid_vels)}")

    if stops < len(mid_vels) * 0.02:  # Less than 2%
        print("✅ SUCCESS: Smooth cruising through waypoints!")
    else:
        print("❌ FAILED: Too many stops")
    print()

    # Plot
    plot_results(t, sine_positions, actual_times, actual_positions,
                 actual_velocities, actual_accelerations)

def plot_results(sine_t, sine_pos, actual_t, actual_pos, actual_vel, actual_acc):
    """Plot comparison"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))

    # Position comparison
    axes[0].plot(sine_t, sine_pos, 'b-', linewidth=3, alpha=0.7, label='Desired Sine Wave')
    axes[0].plot(actual_t, actual_pos, 'r-', linewidth=2, label='Ruckig Trajectory')
    axes[0].scatter(sine_t[::10], sine_pos[::10], c='blue', s=20, alpha=0.4, zorder=5, label='Waypoints')
    axes[0].set_xlabel('Time (s)', fontsize=12)
    axes[0].set_ylabel('Position', fontsize=12)
    axes[0].set_title('Position: Desired vs Actual (THESE SHOULD OVERLAP!)', fontsize=14, fontweight='bold')
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)

    # Velocity
    axes[1].plot(actual_t, actual_vel, 'g-', linewidth=2, label='Velocity')
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[1].fill_between(actual_t, -0.01, 0.01, alpha=0.2, color='red', label='Stop zone (|v| < 0.01)')
    axes[1].set_xlabel('Time (s)', fontsize=12)
    axes[1].set_ylabel('Velocity (m/s)', fontsize=12)
    axes[1].set_title('Velocity (Should NOT stay near zero in middle)', fontsize=14, fontweight='bold')
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)

    # Acceleration
    axes[2].plot(actual_t, actual_acc, 'purple', linewidth=1.5, label='Acceleration')
    axes[2].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].set_ylabel('Acceleration (m/s²)', fontsize=12)
    axes[2].set_title('Acceleration', fontsize=14, fontweight='bold')
    axes[2].legend(fontsize=10)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()

    output_path = '/home/yuuki/ROS2/test_ruckig_sinus/batch_test_result.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")
    print()
    print("=" * 70)
    print("CHECK THE PLOT:")
    print("  Blue sine wave and Red Ruckig trajectory should OVERLAP!")
    print("  If they don't overlap = Ruckig not following waypoints correctly")
    print("=" * 70)

if __name__ == '__main__':
    test_batch_waypoints()
