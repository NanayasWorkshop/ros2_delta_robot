import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicHermiteSpline
from scipy.integrate import solve_ivp

# Waypoints
waypoints = np.array([-3.0, -7.0, -10.0, -7.0, 3.0, 5.0, 3.0, 5.0, 7.0, 4.0])
num_waypoints = len(waypoints)

print("Waypoints:", waypoints)

# Create smooth Hermite spline
u_waypoints = np.arange(num_waypoints)
velocities_at_waypoints = np.zeros(num_waypoints)

for i in range(num_waypoints):
    if i == 0:
        velocities_at_waypoints[i] = (waypoints[i+1] - waypoints[i])
    elif i == num_waypoints - 1:
        velocities_at_waypoints[i] = (waypoints[i] - waypoints[i-1])
    else:
        velocities_at_waypoints[i] = (waypoints[i+1] - waypoints[i-1]) / 2.0

hermite_spline = CubicHermiteSpline(u_waypoints, waypoints, velocities_at_waypoints)

# Sample for visualization
u_dense = np.linspace(0, num_waypoints-1, 500)
spline_positions = hermite_spline(u_dense)

# Your constraints
v_max = 2.0   # mm/s in joint space
a_max = 10.0  # mm/s² in joint space

# Compute path velocity limit at each point
# v_path = how fast we move along path parameter u
# v_joint = |dp/du| * v_path
# So: v_path_max = v_max / |dp/du|

u_samples = np.linspace(0, num_waypoints-1, 1000)
dp_du = hermite_spline.derivative()(u_samples)  # Derivative of position w.r.t. u

# Maximum path velocity at each point (avoid division by zero)
v_path_max = np.abs(v_max / (np.abs(dp_du) + 1e-6))

# Similarly for acceleration (simplified - assumes constant path velocity locally)
a_path_max = np.abs(a_max / (np.abs(dp_du) + 1e-6))

# For now: use simple constant velocity profile (we can improve this)
# Time to traverse path at maximum safe velocity
dt = np.diff(u_samples) / v_path_max[:-1]
t_samples = np.concatenate([[0], np.cumsum(dt)])
total_time = t_samples[-1]

print(f"\nTotal trajectory time: {total_time:.3f} seconds")

# Now sample the trajectory at control rate
control_rate = 100  # Hz
t_control = np.linspace(0, total_time, int(total_time * control_rate))

# Interpolate to get u(t)
u_traj = np.interp(t_control, t_samples, u_samples)

# Get actual positions, velocities, accelerations
positions_traj = hermite_spline(u_traj)
velocities_traj = hermite_spline.derivative()(u_traj) * np.gradient(u_traj, t_control)
accelerations_traj = np.gradient(velocities_traj, t_control)

# Plot everything
plt.figure(figsize=(12, 10))

# Subplot 1: Hermite spline
plt.subplot(4, 1, 1)
plt.plot(u_dense, spline_positions, 'b-', linewidth=2, label='Hermite Spline')
plt.scatter(u_waypoints, waypoints, color='red', s=100, zorder=5, label='Waypoints')
plt.ylabel('Position [mm]')
plt.title('Hermite Spline Geometry')
plt.legend()
plt.grid(True)

# Subplot 2: Position trajectory
plt.subplot(4, 1, 2)
plt.plot(t_control, positions_traj, 'b-', linewidth=2, label='Our Trajectory')
waypoint_times = np.linspace(0, total_time, num_waypoints)
plt.scatter(waypoint_times, waypoints, color='red', s=100, zorder=5, label='Waypoints')
plt.ylabel('Position [mm]')
plt.title('Position Trajectory (EXACT HERMITE SPLINE)')
plt.legend()
plt.grid(True)

# Subplot 3: Velocity
plt.subplot(4, 1, 3)
plt.plot(t_control, velocities_traj, 'b-', linewidth=2)
plt.axhline(y=v_max, color='r', linestyle='--', label=f'Limit: ±{v_max} mm/s')
plt.axhline(y=-v_max, color='r', linestyle='--')
plt.ylabel('Velocity [mm/s]')
plt.title('Velocity')
plt.legend()
plt.grid(True)

# Subplot 4: Acceleration
plt.subplot(4, 1, 4)
plt.plot(t_control, accelerations_traj, 'b-', linewidth=2)
plt.axhline(y=a_max, color='r', linestyle='--', label=f'Limit: ±{a_max} mm/s²')
plt.axhline(y=-a_max, color='r', linestyle='--')
plt.ylabel('Acceleration [mm/s²]')
plt.xlabel('Time [s]')
plt.title('Acceleration')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# Verification
print("\n=== Verification ===")
print(f"Max velocity: {np.max(np.abs(velocities_traj)):.3f} mm/s (limit: {v_max})")
print(f"Max acceleration: {np.max(np.abs(accelerations_traj)):.3f} mm/s² (limit: {a_max})")

# Check waypoint accuracy
errors = []
for i, wp_time in enumerate(waypoint_times):
    idx = np.argmin(np.abs(t_control - wp_time))
    error = abs(positions_traj[idx] - waypoints[i])
    errors.append(error)
    print(f"Waypoint {i}: target={waypoints[i]:.2f}, actual={positions_traj[idx]:.2f}, error={error:.4f} mm")

print(f"\nMax waypoint error: {max(errors):.4f} mm")
