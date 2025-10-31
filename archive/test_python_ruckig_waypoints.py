#!/usr/bin/env python3
"""Test Python Ruckig with waypoints/intermediate_positions"""

from ruckig import Ruckig, InputParameter, OutputParameter, Result

def test_waypoints():
    print("Testing Python Ruckig with Waypoints")
    print("=" * 50)

    # Setup
    control_cycle = 0.01
    dofs = 3
    max_waypoints = 10

    # Create Ruckig instance with waypoint support
    ruckig = Ruckig(dofs, control_cycle, max_waypoints)
    input_param = InputParameter(dofs)
    output_param = OutputParameter(dofs, max_waypoints)

    # Set current state
    input_param.current_position = [0.0, 0.0, 0.0]
    input_param.current_velocity = [0.0, 0.0, 0.0]
    input_param.current_acceleration = [0.0, 0.0, 0.0]

    # Set intermediate waypoints (robot will cruise through these)
    input_param.intermediate_positions = [
        [1.0, 0.5, 0.2],   # Waypoint 1
        [2.0, 1.0, 0.4],   # Waypoint 2
        [3.0, 0.5, 0.3],   # Waypoint 3
    ]

    # Set final target
    input_param.target_position = [4.0, 0.0, 0.0]
    input_param.target_velocity = [0.0, 0.0, 0.0]
    input_param.target_acceleration = [0.0, 0.0, 0.0]

    # Set limits
    input_param.max_velocity = [2.0, 2.0, 2.0]
    input_param.max_acceleration = [5.0, 5.0, 5.0]
    input_param.max_jerk = [10.0, 10.0, 10.0]

    print(f"Starting position: {input_param.current_position}")
    print(f"Waypoints: {len(input_param.intermediate_positions)}")
    for i, wp in enumerate(input_param.intermediate_positions):
        print(f"  Waypoint {i+1}: {wp}")
    print(f"Final target: {input_param.target_position}")
    print()

    # Try to calculate trajectory
    print("Calculating trajectory...")
    try:
        step_count = 0
        max_steps = 10  # Just show first 10 steps

        while True:
            result = ruckig.update(input_param, output_param)

            if step_count < max_steps:
                print(f"Step {step_count}: pos={output_param.new_position}, vel={output_param.new_velocity}")
            elif step_count == max_steps:
                print("... (continuing)")

            step_count += 1

            if result == Result.Finished:
                print(f"\n✅ SUCCESS! Trajectory completed in {step_count} steps")
                print(f"Final position: {output_param.new_position}")
                print(f"Total duration: {output_param.trajectory.duration:.3f} seconds")
                break
            elif result == Result.Error or result == Result.ErrorInvalidInput:
                print(f"\n❌ ERROR: {result}")
                break

            # Pass output to input for next iteration
            output_param.pass_to_input(input_param)

            # Safety limit
            if step_count > 10000:
                print("\n⚠️  Reached max steps")
                break

    except Exception as e:
        print(f"\n❌ Exception: {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "=" * 50)

if __name__ == '__main__':
    test_waypoints()
