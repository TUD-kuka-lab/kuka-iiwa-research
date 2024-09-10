"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

from low_level_controllers.joint_impedance_control import JointImpedanceController
from inverse_kinematics import inverse_kinematics_init
import numpy as np


class CartesianIKController:
    def __init__(self, robot_name):
        self.joint_impedance_controller = JointImpedanceController(robot_name)
        self.joint_impedance_controller.alpha = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]) * 0.2
        self.IK = inverse_kinematics_init('ranged ik', robot_name=robot_name)

    def control_law(self, q, q_dot, pose_d, frequency):
        # Compute inverse kinematics
        q_d = self.IK.compute(pose_d[:3], pose_d[3:], q)

        # Compute q dot
        if frequency != 0:
            q_dot_d = (q_d - q) * frequency
        else:
            q_dot_d = np.array([0, 0, 0, 0, 0, 0 ,0])

        # Get torque from joint impedance controller
        torque = self.joint_impedance_controller.control_law(q=q, q_d=q_d, q_dot_d=q_dot_d, q_dot=q_dot)

        return torque