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

from low_level_controllers.joint_impedance_control import JointImpedanceController
from low_level_controllers.cartesian_impedance_control import CartesianImpedanceController
from low_level_controllers.cartesian_ik_control import CartesianIKController
from iiwa_robotics_toolbox import iiwa
import std_msgs.msg as std_msg
import cor_tud_msgs.msg as cor_msg
from sensor_msgs.msg import JointState
import numpy as np
import rospy
import argparse
from spatialmath import SO3

# Get arguments
parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', type=str, default='default_robot', help='Name of the robot')
args, _ = parser.parse_known_args()


class ControlServer:
    def __init__(self):
        robot_name = args.robot_name
        robot = iiwa(model=robot_name)

        # Create controller
        self.joint_impedance_controller = JointImpedanceController(robot_name=robot_name)
        self.cartesian_impedance_controller = CartesianImpedanceController(robot=robot)
        self.cartesian_ik_controller = CartesianIKController(robot_name=robot_name)

        # Start ROS publishers and subscribers
        self.command_pub = rospy.Publisher('/%s/server_torque_command' % robot_name, std_msg.Float64MultiArray, queue_size=10)
        rospy.Subscriber('/%s/joint_states' % robot_name, JointState, self._callback_joint_states, queue_size=10)
        rospy.Subscriber('/%s/control_request' % robot_name, cor_msg.ControlRequest, self._callback_control_request, queue_size=10) 
        rospy.Subscriber('/%s/end_effector_state' % robot_name, cor_msg.CartesianState, self._callback_end_effector_state, queue_size=10)
        rospy.Subscriber('/%s/elbow_state' % robot_name, cor_msg.CartesianState, self._callback_elbow_state, queue_size=10)

        # Create variables
        self.q = None
        self.q_dot = None
        self.q_d = None
        self.q_dot_d = None
        self.pose_d = None
        self.control_type = None
        self.ee_pose = None
        self.ee_velocity = None
        self.elbow_pose = None
        self.elbow_velocity = None

    def _callback_joint_states(self, data):
        self.q = np.array(data.position)
        self.q_dot = np.array(data.velocity)

    def _callback_end_effector_state(self, data):
        self.ee_pose = np.array(data.pose.data)
        self.ee_velocity = np.array(data.velocity.data)

    def _callback_elbow_state(self, data):
        self.elbow_pose = np.array(data.pose.data)
        self.elbow_velocity = np.array(data.velocity.data)

    def _callback_control_request(self, msg):
        self.q_d = np.array(msg.q_d.data)
        self.q_dot_d = np.array(msg.q_dot_d.data)
        if self.q_dot_d.size == 0:  # if no velocity is sent
            self.q_dot_d = np.array([0, 0, 0, 0, 0, 0 ,0])
        self.pose_d = np.array(msg.pose_d.data)
        self.frequency = msg.frequency.data
        self.control_type = msg.control_type
  
    def send_torque_request(self):
        if self.control_type == 'joint impedance':
            torque = self.joint_impedance_controller.control_law(q=self.q, q_d=self.q_d, q_dot=self.q_dot, q_dot_d=self.q_dot_d)
        elif self.control_type == 'cartesian impedance':
            torque = self.cartesian_impedance_controller.control_law(position_d=self.pose_d[:3], 
                                                                     orientation_d=SO3.RPY(self.pose_d[3:]),
                                                                     ee_pose=self.ee_pose,
                                                                     ee_velocity=self.ee_velocity,
                                                                     elbow_pose=self.elbow_pose,
                                                                     elbow_velocity=self.elbow_velocity,
                                                                     q=self.q)
        elif self.control_type == 'cartesian ik':
            torque = self.cartesian_ik_controller.control_law(q=self.q, q_dot=self.q_dot, pose_d=self.pose_d, frequency=self.frequency)
        elif self.control_type == 'gravity compensation':
            torque = np.zeros(7)

        else:
            print('Control type does not exist')  # TODO: add stay here() (?)

        # Create and publish ROS message
        msg = std_msg.Float64MultiArray()
        msg.data = torque
        self.command_pub.publish(msg)
        return True


if __name__ == '__main__':
    rospy.init_node('control_server')
    control = ControlServer()
    rate = rospy.Rate(1000)
    rate.sleep()  # apparently we need to run this first for the reset to work
    while not rospy.is_shutdown():
        try:
            if control.q_d is None or control.q is None:
                #print('waiting...')
                continue
            control.send_torque_request()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
