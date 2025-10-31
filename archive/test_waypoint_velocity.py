#!/usr/bin/env python3
"""Test if Ruckig maintains velocity through waypoints (not stopping)"""

from ruckig import Ruckig, InputParameter, OutputParameter, Result

def test_velocity_at_waypoints():
    print("Testing Velocity Through Waypoints")
    print("=" * 60)

    control_cycle = 0.01
    dofs = 1  # Use 1 DOF for easier tracking
    max_waypoints = 10

    ruckig = Ruckig(dofs, control_cycle, max_waypoints)
    input_param = InputParameter(dofs)
    output_param = OutputParameter(dofs, max_waypoints)

    # Simple 1D trajectory
    input_param.current_position = [0.0]
    input_param.current_velocity = [0.0]
    input_param.current_acceleration = [0.0]

    # Waypoints at positions 1, 2, 3
    input_param.intermediate_positions = [
        [1.0],
        [2.0],
        [3.0],
    ]

    input_param.target_position = [4.0]
    input_param.target_velocity = [0.0]  # Stop at end
    input_param.target_acceleration = [0.0]

    input_param.max_velocity = [2.0]
    input_param.max_acceleration = [5.0]
    input_param.max_jerk = [10.0]

    print(f"Start: {input_param.current_position[0]}")
    print(f"Waypoints: {[wp[0] for wp in input_param.intermediate_positions]}")
    print(f"End: {input_param.target_position[0]}")
    print()

    # Track positions and velocities
    positions_at_waypoints = {}
    velocities_at_waypoints = {}
    waypoint_targets = [1.0, 2.0, 3.0, 4.0]

    print("Position | Velocity | Near Waypoint?")
    print("-" * 60)

    step = 0
    while True:
        result = ruckig.update(input_param, output_param)

        pos = output_param.new_position[0]
        vel = output_param.new_velocity[0]

        # Check if we're near a waypoint (within 0.01m)
        for wp_pos in waypoint_targets:
            if abs(pos - wp_pos) < 0.01 and wp_pos not in positions_at_waypoints:
                positions_at_waypoints[wp_pos] = pos
                velocities_at_waypoints[wp_pos] = vel
                print(f"{pos:8.4f} | {vel:8.4f} | ← AT WAYPOINT {wp_pos}m")

        # Show every 20 steps
        if step % 20 == 0:
            print(f"{pos:8.4f} | {vel:8.4f} |")

        step += 1

        if result == Result.Finished:
            print()
            print("=" * 60)
            print("RESULTS:")
            print("=" * 60)
            for wp_pos in waypoint_targets:
                if wp_pos in velocities_at_waypoints:
                    vel = velocities_at_waypoints[wp_pos]
                    status = "✅ CRUISING" if abs(vel) > 0.1 else "❌ STOPPED"
                    print(f"Waypoint at {wp_pos}m: velocity = {vel:.4f} m/s {status}")
            print()

            # Check if it truly cruised through
            intermediate_waypoints = [1.0, 2.0, 3.0]
            cruised_through_all = all(
                abs(velocities_at_waypoints.get(wp, 0)) > 0.1
                for wp in intermediate_waypoints
            )

            if cruised_through_all:
                print("✅ SUCCESS: Robot cruised through all intermediate waypoints!")
            else:
                print("❌ FAILED: Robot stopped at waypoints")

            break
        elif result == Result.Error or result == Result.ErrorInvalidInput:
            print(f"\n❌ ERROR: {result}")
            break

        output_param.pass_to_input(input_param)

        if step > 10000:
            break

if __name__ == '__main__':
    test_velocity_at_waypoints()
