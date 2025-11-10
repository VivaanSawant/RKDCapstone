import numpy as np
from FrankaRobot16384 import Franka16384
from frankapy.utils import min_jerk



robot = Franka16384()
robot.reset()
# Get current joints
q_start = robot.get_joints()

# Move to a specific configuration
q_target = q_start + np.array([0.1, 0, 0, 0, 0, 0, 0])
robot.set_joints(q_target)

q_curr = np.array([0.13552955, 0.0635955,   0.16959005, -2.29884704 , 0.00745588  ,2.47181028,
  1.18640869])

robot.set_joints(q_curr)


q_target = np.array([ 0.02539866,  0.13048301 ,-0.06443245, -2.08149549,  0.01439907,  2.09092226,
  0.74839636])
# Simple trajectory
T = 10
dt = 0.05

#generates a min-jerk trajectory from q_curr to q_target , here T = 10 which is total time and dt = 0.05 is time step
# T is total time for the trajectory.
# dt is time step for publishing and moving.
# you will be generateing an interpolated trajectory between q_curr and q_target using some trajectory generation method like min-jerk 
# could be trapezodial velocity profile or Slerp etc
traj = [min_jerk(q_curr, q_target, t, T) for t in np.arange(0, T, dt)]
robot.follow_trajectory(traj, T=10, dt=0.05)

# Gripper control
robot.close_gripper()
robot.open_gripper()

robot.reset()