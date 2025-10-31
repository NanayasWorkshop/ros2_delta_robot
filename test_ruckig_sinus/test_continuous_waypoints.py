#!/usr/bin/env python3
"""
Test Ruckig with continuous waypoint stream (like ROS system).
Simulates waypoints arriving in a buffer, should cruise through them.
"""

import numpy as np
import matplotlib.pyplot as plt
from ruckig import Ruckig, InputParameter, OutputParameter, Result

def generate_sine_waypoints(frequency=0.5, duration=2.0, sample_rate=50):
    """Generate sine wave waypoints"""
    t = np.linspace(0, duration, int(duration * sample_rate))
    positions = np.sin(2 * np.pi * frequency * t)
    return positions.tolist()

def test_continuous_waypoint_buffer():
    print("=" * 70)
    print("Testing Ruckig with Continuous Waypoint Buffer (ROS-like)")
    print("=" * 70)

    # Generate ALL waypoints upfront
    all_waypoints = generate_sine_waypoints(
        frequency=0.5,    # 0.5 Hz sine
        duration=4.0,     # 4 seconds (2 full cycles)
        sample_rate=50    # 50 waypoints/second = one every 0.02s
    )

    print(f"Generated {len(all_waypoints)} total waypoints")
    print(f"Waypoint spacing: 0.02s (50 Hz)")
    print()

    # Setup Ruckig
    control_cycle = 0.01  # 100 Hz control
    max_lookahead = 10

    ruckig = Ruckig(1, control_cycle, max_lookahead)
    input_param = InputParameter(1)
    output_param = OutputParameter(1, max_lookahead)

    # Set limits
    input_param.max_velocity = [2.0]
    input_param.max_acceleration = [5.0]
    input_param.max_jerk = [10.0]

    # Initialize
    input_param.current_position = [all_waypoints[0]]
    input_param.current_velocity = [0.0]
    input_param.current_acceleration = [0.0]

    # Simulate buffer - starts with some waypoints
    waypoint_buffer = all_waypoints[1:20].copy()  # Start with 19 waypoints in buffer
    next_waypoint_idx = 20  # Next waypoint to add

    # Track results
    actual_positions = []
    actual_velocities = []
    actual_times = []
    buffer_sizes = []

    current_time = 0.0
    step = 0
    max_steps = 600

    print("Starting simulation...")
    print(f"Initial buffer size: {len(waypoint_buffer)}")
    print()

    while step < max_steps and (len(waypoint_buffer) > 0 or next_waypoint_idx < len(all_waypoints)):
        # Simulate new waypoints arriving (every 0.02s = every 2 control cycles)
        if step % 2 == 0 and next_waypoint_idx < len(all_waypoints):
            waypoint_buffer.append(all_waypoints[next_waypoint_idx])
            next_waypoint_idx += 1

        # Apply lookahead strategy (like motor_trajectory_smoother)
        buffer_size = len(waypoint_buffer)
        lookahead = min(max_lookahead, buffer_size)

        if lookahead == 0:
            break

        if lookahead == 1:
            # Last waypoint - stop here
            input_param.target_position = [waypoint_buffer[0]]
            input_param.intermediate_positions = []
            input_param.target_velocity = [0.0]
        else:
            # Multiple waypoints - cruise through first N-1, target is last
            input_param.intermediate_positions = []
            for i in range(lookahead - 1):
                input_param.intermediate_positions.append([waypoint_buffer[i]])

            input_param.target_position = [waypoint_buffer[lookahead - 1]]
            input_param.target_velocity = [0.0]  # Let Ruckig optimize

        input_param.target_acceleration = [0.0]

        # Update Ruckig
        result = ruckig.update(input_param, output_param)

        # Record
        actual_positions.append(output_param.new_position[0])
        actual_velocities.append(output_param.new_velocity[0])
        actual_times.append(current_time)
        buffer_sizes.append(buffer_size)

        # Print every 50 steps
        if step % 50 == 0:
            print(f"Step {step:3d} | Time: {current_time:.2f}s | "
                  f"Pos: {output_param.new_position[0]:6.3f} | "
                  f"Vel: {output_param.new_velocity[0]:6.3f} | "
                  f"Buffer: {buffer_size:2d} | "
                  f"Lookahead: {len(input_param.intermediate_positions)}+1")

        # If reached target, remove processed waypoints
        if result == Result.Finished:
            # Remove waypoints up to target
            if lookahead == 1:
                waypoint_buffer.pop(0)
            else:
                # Remove all but last in lookahead (target becomes new current)
                for _ in range(lookahead - 1):
                    if waypoint_buffer:
                        waypoint_buffer.pop(0)

        # Pass output to input
        output_param.pass_to_input(input_param)

        current_time += control_cycle
        step += 1

    print()
    print("=" * 70)
    print("Results:")
    print("=" * 70)
    print(f"Total steps: {step}")
    print(f"Total time: {current_time:.2f}s")
    print(f"Waypoints processed: {next_waypoint_idx}/{len(all_waypoints)}")
    print()

    # Analyze
    velocities_array = np.array(actual_velocities)
    print("Velocity Analysis:")
    print(f"  Max velocity: {np.max(np.abs(velocities_array)):.3f} m/s")
    print(f"  Mean velocity: {np.mean(np.abs(velocities_array)):.3f} m/s")

    # Count stops (exclude first/last 10% for acceleration/deceleration)
    mid_start = len(velocities_array) // 10
    mid_end = len(velocities_array) - mid_start
    mid_velocities = velocities_array[mid_start:mid_end]
    stopped_count = np.sum(np.abs(mid_velocities) < 0.01)
    print(f"  Times stopped in middle (|v| < 0.01): {stopped_count}/{len(mid_velocities)}")

    if stopped_count > len(mid_velocities) * 0.05:
        print("  ❌ FAILED: Too many stops!")
    else:
        print("  ✅ SUCCESS: Smooth cruising!")

    # Plot
    plot_continuous_results(all_waypoints, actual_positions, actual_velocities,
                           actual_times, buffer_sizes)

def plot_continuous_results(all_waypoints, actual_positions, actual_velocities,
                            actual_times, buffer_sizes):
    """Plot results"""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

    # Position
    waypoint_times = np.linspace(0, 4.0, len(all_waypoints))
    ax1.plot(waypoint_times, all_waypoints, 'b--', linewidth=2, alpha=0.6, label='Desired (waypoints)')
    ax1.plot(actual_times, actual_positions, 'r-', linewidth=1.5, label='Actual Ruckig')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position')
    ax1.set_title('Position Tracking')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Velocity
    ax2.plot(actual_times, actual_velocities, 'g-', linewidth=1.5)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Velocity (should stay non-zero for cruising)')
    ax2.grid(True, alpha=0.3)

    # Buffer size
    ax3.plot(actual_times, buffer_sizes, 'purple', linewidth=1.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Buffer Size')
    ax3.set_title('Waypoint Buffer Size Over Time')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('/home/yuuki/ROS2/test_ruckig_sinus/continuous_test.png', dpi=150)
    print()
    print("Plot saved to: /home/yuuki/ROS2/test_ruckig_sinus/continuous_test.png")

if __name__ == '__main__':
    test_continuous_waypoint_buffer()
