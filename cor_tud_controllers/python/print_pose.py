#!/usr/bin/python3

"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

from sensor_msgs.msg import JointState
from iiwa_robotics_toolbox import iiwa
from spatialmath import UnitQuaternion
import numpy as np
import rospy
import sys

ROBOT = ROBOT = sys.argv[1]

class CartesianStatePrinter:
    def __init__(self):
        self.q = None
        self.q_dot = None
        self.robot = iiwa(model=ROBOT)
        rospy.Subscriber('/%s/joint_states' % ROBOT, JointState, self._callback_joint_states, queue_size=10)

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

        # Transform orientation matrix to quaternion
        orientation_quaternion = UnitQuaternion(orientation_end_effector)

        return position_end_effector, orientation_quaternion

    def _print_cartesian_state(self, end_link):
        position, orientation = self._get_cartesian_state(end_link=end_link)
        print('Position %s:' % end_link, position)
        print('Oriantation unit quaternion %s:' % end_link, orientation)
        print('Oriantation RPY %s:' % end_link, orientation.rpy())
        print('\n')
        return True

    def run(self):
        rospy.sleep(0.1)

        # Publish end effector's state
        self._print_cartesian_state(end_link='iiwa_link_7')

        # Publish elbow's state
        self._print_cartesian_state(end_link='iiwa_link_3')


if __name__ == '__main__':
    # Init ROS
    rospy.init_node('print_cartesian_state')
    control_server = CartesianStatePrinter()
    control_server.run()
