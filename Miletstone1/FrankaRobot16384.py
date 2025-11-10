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

    def get_joints(self):
        """Return the current joint configuration."""
        return self.fa.get_joints()

    def set_joints(self, q):
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

    def follow_trajectory(self,joints_traj, T=5, dt=0.02):
            """
            Follow a trajectory based on a min-jerk interpolation of joint angles.
            joints_0: initial joint configuration.
            joints_1: final joint configuration.
            T: total time for the trajectory.
            dt: time step for publishing and moving.
            """
            # Generate the joint trajectory using min-jerk interpolation
            ts = np.arange(0, T, dt)
            

            # ROS setup for publishing joint positions
            rospy.loginfo('Initializing Sensor Publisher')
            pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
            rate = rospy.Rate(1 / dt)  # Frequency to send data

            rospy.loginfo('Publishing joints trajectory...')
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
                rospy.loginfo(f'Publishing: ID {traj_gen_proto_msg.id}')
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

            rospy.loginfo('Done')
            print("[Franka16384] Trajectory execution complete.")