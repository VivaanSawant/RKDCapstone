import numpy as np
import time
import rospy
from frankapy import FrankaConstants as FC
from frankapy import FrankaArm,SensorDataMessageType
from frankapy.proto_utils import make_sensor_group_msg, sensor_proto2ros_msg

from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk

class Franka16384:
    def __init__(self):
        """Initialize the Franka robot using FrankaPy."""
        self.fa = FrankaArm()

    # -----------------------------
    # Joint getters / setters
    # -----------------------------
    def reset(self):
        """Reset the robot to its home position."""
        self.fa.reset_joints()
        print("[Franka16384] Robot reset to home position.")

    def get_config(self):
        """Return the current joint configuration."""
        return self.fa.get_joints()

    def set_config(self, q):
        """
        Move to a specific joint configuration.
        q: np.array(7,) or list of 7 joint values.
        """

        q = np.array(q)
        assert q.shape == (7,)
        self.fa.goto_joints(q)
        print("[Franka16384] Reached target joint configuration.")

    # -----------------------------
    # Gripper control
    # -----------------------------
    def open_gripper(self):
        """Open the gripper fully."""
        self.fa.open_gripper()
        print("[Franka16384] Gripper opened.")

    def close_gripper(self):
        """Close the gripper fully."""
        self.fa.close_gripper()
        print("[Franka16384] Gripper closed.")

    # -----------------------------
    # Trajectory execution
    # -----------------------------

    def follow_trajectory(self, joints_traj, T=2):
        """
        Follow a trajectory based on a min-jerk interpolation of joint angles.
        joints_traj: A list of joint configurations.
        T: Total time for the trajectory.
        """

        # Fixed time step dt
        dt = 0.02

        # 1. Check if joints_traj is a list/array of joint configurations and its length is valid.
        assert isinstance(joints_traj, (list, np.ndarray)), "joints_traj must be a list or ndarray."
        assert len(joints_traj) > 1, "joints_traj must contain at least two joint configurations."

        # 2. Check that each waypoint has exactly 7 joint angles.
        for i, joint_config in enumerate(joints_traj):
            assert len(joint_config) == 7, f"Waypoint {i} does not have exactly 7 joint angles: {joint_config}"

        # 3. Check if T is valid (positive and reasonable).
        assert T > 0, "Total time T must be positive."

        # 4. Validate that dt is reasonable (positive and not too small).
        assert dt > 0, "Time step dt must be positive."
        assert dt < T, "Time step dt must be smaller than total time T."

        # 5. Ensure start joint configuration matches the current robot state (within some buffer).
        current_joint_state = self.get_config()  # assuming this function fetches the current joint state
        assert np.allclose(joints_traj[0], current_joint_state, atol=0.05), \
            f"Initial joint configuration does not match current state. Difference: {np.abs(joints_traj[0] - current_joint_state)}"

        # 6. Check that all joints are within the reachable limits.
        for idx, joint in enumerate(joints_traj):
            if not self.fa.is_joints_reachable(joint):
                raise ValueError(f"Joint configuration {joint} is out of bounds (joint {idx}).")

        # 7. Check that the deltas between consecutive joint positions are within velocity limits.
        max_velocity = 1.0  # Placeholder, assuming you have a limit on velocity (rad/s or units).
        for i in range(1, len(joints_traj)):
            delta_position = np.abs(joints_traj[i] - joints_traj[i - 1])
            velocity = delta_position / dt
            rospy.loginfo(f"Velocity between steps {i-1} and {i}: {velocity}")
            assert np.all(velocity <= max_velocity), f"Joint velocity exceeds limit between steps {i-1} and {i}. Velocity: {velocity}"

        # 8. Verify the robot is within its virtual workspace before running.
        for joint_config in joints_traj:
            if self.fa.is_joints_in_collision_with_boxes(joints=joint_config):
                raise ValueError("Trajectory would result in a collision.")

        # Generate the joint trajectory using min-jerk interpolation
        ts = np.arange(0, T, dt)

        # ROS setup for publishing joint positions
        # rospy.loginfo('Initializing Sensor Publisher')
        pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
        rate = rospy.Rate(1 / dt)  # Frequency to send data

        # rospy.loginfo('Publishing joints trajectory...')
        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        self.fa.goto_joints(joints_traj[1], duration=T, dynamic=True, buffer_time=10)
        init_time = rospy.Time.now().to_time()

        # Iterate over the trajectory and execute each joint configuration
        for i in range(2, len(ts)):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i,
                timestamp=rospy.Time.now().to_time() - init_time,
                joints=joints_traj[i]
            )

            # Create and publish the ROS message
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            # rospy.loginfo(f'Publishing: ID {traj_gen_proto_msg.id}')
            pub.publish(ros_msg)

            # Move to the next joint configuration
            # self.fa.goto_joints(joints_traj[i], duration=dt, dynamic=True)

            # Wait for the next step (time interval between waypoints)
            rate.sleep()

        # Stop the trajectory and notify the system
        term_proto_msg = ShouldTerminateSensorMessage(
            timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True
        )
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
        pub.publish(ros_msg)

        # rospy.loginfo('Done')
        print("[Franka16384] Trajectory execution complete.")