#!/usr/bin/python3

"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""
# Add path with iiwa_robotics_toolbox file
import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
sys.path.append(parent_dir)

# Imports
import std_msgs.msg as std_msg
import cor_tud_msgs.msg as cor_msg
from sensor_msgs.msg import JointState
from iiwa_robotics_toolbox import iiwa
from spatialmath import SO3
import numpy as np
import rospy
import argparse

# Get arguments
parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', type=str, default='default_robot', help='Name of the robot')
args, _ = parser.parse_known_args()


class CartesianStatePublisher:
    def __init__(self, control_frequency):
        self.control_frequency = control_frequency
        self.q = None
        self.q_dot = None
        robot_name = args.robot_name
        self.robot = iiwa(model=robot_name)
        self.end_effector_state_pub = rospy.Publisher('/%s/end_effector_state' % robot_name, cor_msg.CartesianState, queue_size=10)
        self.elbow_state_pub = rospy.Publisher('/%s/elbow_state' % robot_name, cor_msg.CartesianState, queue_size=10)
        self.wrist_state_pub = rospy.Publisher('/%s/wrist_state' % robot_name, cor_msg.CartesianState, queue_size=10)
        rospy.Subscriber('/%s/joint_states' % robot_name, JointState, self._callback_joint_states, queue_size=10)

    def _callback_joint_states(self, data):
        self.q = np.array(data.position)
        self.q_dot = np.array(data.velocity)

    def _get_pose(self, end_link='iiwa_link_7'):
        pose = self.robot.fkine(self.q, end=end_link, start='iiwa_link_0')
        position = pose.t
        orientation = pose.R
        return position, orientation

    def _get_cartesian_state(self, end_link='iiwa_link_7'):
        # Get pose
        position_end_effector, orientation_end_effector = self._get_pose(end_link=end_link)

        # Transform orientation matrix to euler
        orientation_rpy_end_effector = SO3(orientation_end_effector).rpy()

        pose = np.append(position_end_effector, orientation_rpy_end_effector)

        # Get velocity
        link_number = int(end_link.split('_')[-1])
        velocity = np.matmul(self.robot.jacob0(self.q, end=end_link, start='iiwa_link_0'), self.q_dot[:link_number])

        return pose, velocity

    def _create_state_msg(self, pose, velocity):
        # Create pose and pose velocity vectors
        pose_msg = std_msg.Float64MultiArray()
        pose_msg.data = pose

        pose_velocity_msg = std_msg.Float64MultiArray()
        pose_velocity_msg.data = velocity

        # Create ROS message to be published
        state_msg = cor_msg.CartesianState()
        state_msg.pose = pose_msg
        state_msg.velocity = pose_velocity_msg

        return state_msg

    def _publish_cartesian_state(self, publisher, end_link):
        pose, velocity = self._get_cartesian_state(end_link=end_link)
        state_msg = self._create_state_msg(pose, velocity)
        publisher.publish(state_msg)
        return True

    def run(self):
        rate = rospy.Rate(self.control_frequency)
        rospy.sleep(0.1)
        while not rospy.is_shutdown():
            try:
                if self.q is None:
                    continue
                # Publish end effector's state
                self._publish_cartesian_state(self.end_effector_state_pub, end_link='iiwa_link_7')

                # Publish elbow's state
                self._publish_cartesian_state(self.elbow_state_pub, end_link='iiwa_link_3')

                # Publish wrist's state
                #self._publish_cartesian_state(self.wrist_state_pub, end_link='iiwa_link_6')
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == '__main__':
    # Init ROS
    rospy.init_node('cartesian_state_publisher')
    control_frequency = 1000
    control_server = CartesianStatePublisher(control_frequency)
    control_server.run()
