#!/usr/bin/python3

"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
    Willem Momma <w.j.momma@student.tudelft.nl>
"""

from sensor_msgs.msg import JointState
from iiwa_robotics_toolbox import iiwa
from spatialmath import UnitQuaternion
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
import numpy as np
import rospy
import sys
import json

class CartesianStatePrinter:
    def __init__(self):
        self.q_iiwa14 = None
        self.q_dot_iiwa14 = None
        self.effort_iiwa14 = None  
        self.robot_iiwa14 = iiwa(model='iiwa14')

        rospy.Subscriber('/CartesianImpedanceController/joint_states', JointState, self._callback_joint_states, queue_size=10)

    def _callback_pose(self, data):
        self.pose = data

    def _callback_joint_states(self, data):
        self.q_iiwa14 = np.array(data.position)
        self.q_dot_iiwa14 = np.array(data.velocity)
        self.effort_iiwa14 = np.array(data.effort)  

    def _get_pose(self, end_link='iiwa_link_tool', robot_model='iiwa7'):
        if robot_model == 'iiwa7':
            self.robot = self.robot_iiwa7
            self.q = self.q_iiwa7
        elif robot_model == 'iiwa14':
            self.robot = self.robot_iiwa14
            self.q = self.q_iiwa14
        pose = self.robot.fkine(self.q, end=end_link, start='iiwa_link_0')
        position = pose.t
        orientation = pose.R
        return position, orientation

    def _get_cartesian_state(self, end_link='iiwa_link_tool', robot_model='iiwa7'):
        # Get pose
        position_end_effector, orientation_end_effector = self._get_pose(end_link=end_link, robot_model=robot_model)

        # Transform orientation matrix to quaternion
        orientation_quaternion = UnitQuaternion(orientation_end_effector)

        return position_end_effector, orientation_quaternion

    def _print_cartesian_state(self, end_link, robot_model):
        positions = []
        orientations = []
        if robot_model == 'iiwa7':
            position, orientation = self._get_cartesian_state(end_link=end_link, robot_model='iiwa7')
        elif robot_model == 'iiwa14':
            for i in range(250):
                position, orientation = self._get_cartesian_state(end_link=end_link, robot_model='iiwa14')
                positions.append(position)
                orientations.append(orientation)
        else:
            raise ValueError('Invalid robot model') 
        meanposition = np.mean(positions, axis=0)
        meanorientation = np.mean(orientations, axis=0)       
        print('Kuka Position %s:' % end_link, meanposition)
        print('Kuka Orientation Quaternion %s:' % end_link, meanorientation)
        print('\n')
        return meanposition.tolist(), str(meanorientation)

    def run(self):
        rospy.sleep(0.1)
        if self.q_iiwa14 is not None:
            position_tool_iiwa14, _ = self._print_cartesian_state(end_link='iiwa_link_tool', robot_model='iiwa14')


if __name__ == '__main__':
    rospy.init_node('print_cartesian_state')
    control_server = CartesianStatePrinter()
    control_server.run()