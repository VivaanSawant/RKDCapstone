import numpy as np
from FrankaRobot16384 import Franka16384       # Wrapper class that interfaces with the physical robot
from milestone_3 import Robot                  # Student's motion planning class (from previous milestone)
from frankapy.utils import min_jerk            # Example interpolation function (for reference only)



# Initialize both robot interfaces
franka = Franka16384()   # Hardware controller (FrankaPy)
planner = Robot()        # Motion planning class (student implementation)

# Reset robot to home position
franka.reset()

# Get current joint state
q_start = franka.get_joints()

# Move to an initial configuration
q_curr = np.array([
    0.13552955, 0.0635955, 0.16959005, -2.29884704,
    0.00745588, 2.47181028, 1.18640869
])
franka.set_joints(q_curr)

# Define target joint configuration
q_target = np.array([
    0.02539866,  0.13048301, -0.06443245, -2.08149549,
    0.01439907,  2.09092226,  0.74839636
])



## Jacovian Verification ##

J_analytical = planner.compute_jacobian_analytical(q_curr)
J_numerical = planner.compute_jacobian_numerical(q_curr)

print("Analytical Jacobian:\n", J_analytical)
print("Numerical Jacobian:\n", J_numerical)
print("Difference (Analytical - Numerical):\n", J_analytical - J_numerical)


# --------------------------------------------------------------------------
# Trajectory generation parameters
# --------------------------------------------------------------------------
# The robot controller receives new waypoints every 0.02 seconds.
# Your interpolation function must ensure:
#   - Waypoints are spaced exactly 0.02 seconds apart.
#   - Each successive waypoint is reachable within 0.02 seconds
#     from the previous one, without exceeding joint limits.
#
# Therefore:
#     dt = 0.02  # time step
#     T  = 2.0   # total motion duration
#     num_waypoints = T / dt
#
# Your trajectory should have num_waypoints waypoints, each 7 joint angles long.
# --------------------------------------------------------------------------

T = planner.total_time  # you can adjust this value as needed in the constructor of Robot class
dt = 0.02         
num_points = int(T / dt)



# --------------------------------------------------------------------------
# TODO: Trajectory Generation (Student Implementation)
# --------------------------------------------------------------------------
# Replace the example below with your own interpolation function
# implemented in the Robot class (e.g., trapezoidal velocity profile).
#
# Requirements:
#   - traj[0] must equal the robot's current joint configuration
#   - Waypoints must be spaced 0.02 s apart (dt)
#   - Total number of waypoints should be num_points
#   - Each waypoint must be reachable from the previous one
#     without exceeding joint limits or velocity limit


# Example (for reference only):
traj = [min_jerk(q_curr, q_target, t, T) for t in np.arange(0, T, dt)]
#
# Replace with your implementation:
# traj = planner.lerp_trajectory(q_curr, q_target,num_waypoints)
# --------------------------------------------------------------------------

# Send the generated trajectory to the robot for execution
franka.follow_trajectory(traj, T=T)

# Optional: Visualize the trajectory
robot.plot_end_effector_trajectory(traj)




# Example gripper operations
franka.close_gripper()
franka.open_gripper()


# Reset the robot to its home position after execution
franka.reset()
