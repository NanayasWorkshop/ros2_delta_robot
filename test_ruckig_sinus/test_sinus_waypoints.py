#!/usr/bin/env python3
"""
Test Ruckig waypoint cruising with continuous sine wave points.
Should smoothly follow sine curve without stopping at each point.
"""

import numpy as np
import matplotlib.pyplot as plt
from ruckig import Ruckig, InputParameter, OutputParameter, Result

def generate_sine_waypoints(frequency=0.5, duration=2.0, sample_rate=200):
    """
    Generate sine wave waypoints.

    Args:
        frequency: Sine wave frequency in Hz (0.5 Hz = one cycle per 2 seconds)
        duration: Total duration in seconds
        sample_rate: Waypoints per second (200 Hz = 200 points/second)

    Returns:
        List of positions
    """
    t = np.linspace(0, duration, int(duration * sample_rate))
    positions = np.sin(2 * np.pi * frequency * t)
    return positions.tolist(), t.tolist()

def test_ruckig_waypoint_cruising():
    print("=" * 70)
    print("Testing Ruckig Waypoint Cruising with Sine Wave")
    print("=" * 70)

    # Generate sine waypoints
    sine_positions, time_points = generate_sine_waypoints(
        frequency=0.5,      # 0.5 Hz sine wave
        duration=2.0,       # 2 seconds (one full cycle)
        sample_rate=200     # 200 waypoints/second
    )

    print(f"Generated {len(sine_positions)} waypoints")
    print(f"Sine frequency: 0.5 Hz")
    print(f"Sample rate: 200 Hz")
    print()

    # Setup Ruckig
    control_cycle = 0.01  # 100 Hz control
    dofs = 1
    max_waypoints = 50  # Look ahead up to 50 waypoints

    ruckig = Ruckig(dofs, control_cycle, max_waypoints)
    input_param = InputParameter(dofs)
    output_param = OutputParameter(dofs, max_waypoints)

    # Set limits
    input_param.max_velocity = [2.0]
    input_param.max_acceleration = [5.0]
    input_param.max_jerk = [10.0]

    # Initialize at first point
    input_param.current_position = [sine_positions[0]]
    input_param.current_velocity = [0.0]
    input_param.current_acceleration = [0.0]

    # Track results
    actual_positions = []
    actual_velocities = []
    actual_times = []
    waypoints_at_time = []

    waypoint_idx = 1  # Start from second waypoint
    current_time = 0.0

    print("Starting trajectory following...")
    print()

    step = 0
    max_steps = 500

    while waypoint_idx < len(sine_positions) and step < max_steps:
        # Determine how many waypoints to feed (lookahead strategy)
        remaining_waypoints = len(sine_positions) - waypoint_idx
        lookahead = min(max_waypoints, remaining_waypoints)

        if lookahead == 0:
            # No more waypoints, stop
            break

        # Strategy: cruise through all but last waypoint in lookahead
        if lookahead == 1:
            # Only 1 waypoint left - this is final target, stop here
            input_param.target_position = [sine_positions[waypoint_idx]]
            input_param.intermediate_positions = []
            input_param.target_velocity = [0.0]  # Stop at end
            input_param.target_acceleration = [0.0]
            is_final = True
        else:
            # Multiple waypoints - cruise through first N-1, target is last
            input_param.intermediate_positions = []
            for i in range(lookahead - 1):
                wp_idx = waypoint_idx + i
                input_param.intermediate_positions.append([sine_positions[wp_idx]])

            # Target is the last waypoint in lookahead
            target_idx = waypoint_idx + lookahead - 1
            input_param.target_position = [sine_positions[target_idx]]
            input_param.target_velocity = [0.0]  # Let Ruckig optimize
            input_param.target_acceleration = [0.0]
            is_final = False

        # Update Ruckig
        result = ruckig.update(input_param, output_param)

        # Record trajectory
        actual_positions.append(output_param.new_position[0])
        actual_velocities.append(output_param.new_velocity[0])
        actual_times.append(current_time)
        waypoints_at_time.append(waypoint_idx)

        # Print progress every 50 steps
        if step % 50 == 0:
            num_intermediate = len(input_param.intermediate_positions)
            print(f"Step {step:3d} | Time: {current_time:.2f}s | "
                  f"Pos: {output_param.new_position[0]:6.3f} | "
                  f"Vel: {output_param.new_velocity[0]:6.3f} | "
                  f"Waypoint: {waypoint_idx}/{len(sine_positions)} | "
                  f"Lookahead: {num_intermediate} intermediate + 1 target")

        # Check if we reached target
        if result == Result.Finished:
            # Move to next waypoint
            if lookahead == 1:
                waypoint_idx += 1
            else:
                waypoint_idx += lookahead - 1

            if step % 50 != 0:  # Print if not already printed
                print(f"  → Reached waypoint {waypoint_idx - 1}")

        # Pass output to input for next cycle
        output_param.pass_to_input(input_param)

        current_time += control_cycle
        step += 1

    print()
    print("=" * 70)
    print("Results:")
    print("=" * 70)
    print(f"Total steps: {step}")
    print(f"Total time: {current_time:.2f}s")
    print(f"Waypoints processed: {waypoint_idx}/{len(sine_positions)}")
    print()

    # Analyze velocity at waypoints
    print("Velocity Analysis:")
    velocities_array = np.array(actual_velocities)
    print(f"  Max velocity: {np.max(np.abs(velocities_array)):.3f} m/s")
    print(f"  Mean velocity: {np.mean(np.abs(velocities_array)):.3f} m/s")
    print(f"  Min velocity: {np.min(np.abs(velocities_array)):.3f} m/s")

    # Check if it stopped (velocity near zero) at intermediate points
    stopped_count = np.sum(np.abs(velocities_array[10:-10]) < 0.01)  # Exclude start/end
    print(f"  Times stopped (|v| < 0.01): {stopped_count} out of {len(velocities_array) - 20}")

    if stopped_count > len(velocities_array) * 0.1:
        print("  ❌ FAILED: Robot stopped too many times!")
    else:
        print("  ✅ SUCCESS: Robot cruised smoothly!")

    # Plot results
    plot_results(sine_positions, time_points, actual_positions, actual_velocities, actual_times)

def plot_results(sine_positions, sine_times, actual_positions, actual_velocities, actual_times):
    """Plot desired vs actual trajectory"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # Position plot
    ax1.plot(sine_times, sine_positions, 'b--', linewidth=2, label='Desired Sine Wave (waypoints)', alpha=0.6)
    ax1.plot(actual_times, actual_positions, 'r-', linewidth=1, label='Actual Ruckig Trajectory')
    ax1.scatter(sine_times[::20], sine_positions[::20], c='blue', s=30, alpha=0.3, label='Waypoint samples')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position')
    ax1.set_title('Ruckig Waypoint Following - Position')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Velocity plot
    ax2.plot(actual_times, actual_velocities, 'g-', linewidth=1.5, label='Actual Velocity')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Ruckig Waypoint Following - Velocity (should NOT drop to zero often)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('/home/yuuki/ROS2/test_ruckig_sinus/ruckig_sine_test.png', dpi=150)
    print()
    print("Plot saved to: /home/yuuki/ROS2/test_ruckig_sinus/ruckig_sine_test.png")
    print()

if __name__ == '__main__':
    test_ruckig_waypoint_cruising()
