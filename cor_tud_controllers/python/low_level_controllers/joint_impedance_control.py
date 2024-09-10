"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

import numpy as np
from low_level_controllers.parameters.joint_impedance_parameters_safe_iiwa7 import Params as Params_iiwa7
from low_level_controllers.parameters.joint_impedance_parameters_safe_iiwa14 import Params as Params_iiwa14


class JointImpedanceController:
    def __init__(self, robot_name):
        if robot_name == 'iiwa7':
            params = Params_iiwa7()
        elif robot_name == 'iiwa14':
            params = Params_iiwa14()
        else:
            raise NameError('Robot name %s does not exist' % robot_name)
        # Parameters position control
        self.stiffness_pos = params.stiffness_pos
        self.damping_pos = params.damping_pos 

        # Parameters velocity control
        self.stiffness_vel = params.stiffness_vel 
        self.damping_vel = params.damping_vel

        # Smoothing and clipping parameters
        self.alpha = params.alpha 
        self.max_q_delta = params.max_q_delta

        # Init variables
        self.q_d_prev = None
        self.torque_prev = None
        self.q_dot_d_prev = None
        self.parameters_change = True

    def saturate_q_delta(self, q_delta):
        q_delta[np.abs(q_delta) > self.max_q_delta] = self.max_q_delta[np.abs(q_delta) > self.max_q_delta] \
                                                      * np.sign(q_delta)[np.abs(q_delta) > self.max_q_delta]
        return q_delta
    
    def control_law(self, q, q_d, q_dot, q_dot_d=np.zeros(7)):
        # Check velocity control
        if np.linalg.norm(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) - q_dot_d) > 0.01:
            damping = self.damping_vel
            stiffness = self.stiffness_vel
        else:  # position control
            damping = self.damping_pos
            stiffness = self.stiffness_pos
    
        # Smooth signal (exponential smoothing)
        if self.q_d_prev is None:
            self.q_d_prev = q_d

        if self.q_dot_d_prev is None:
            self.q_dot_d_prev = q_dot_d
    
        q_d_filtered = self.alpha * q_d + (1 - self.alpha) * self.q_d_prev
        self.q_d_prev = q_d_filtered

        q_dot_d_filtered = self.alpha * q_dot_d + (1 - self.alpha) * self.q_dot_d_prev
        self.q_dot_d_prev = q_dot_d_filtered

        # Get angle error
        q_delta = q_d_filtered - q

        # Saturate error if too large
        q_delta_saturated = self.saturate_q_delta(q_delta)

        # Get angle velocity error
        q_dot_delta = q_dot_d_filtered - q_dot

        # Compute desired torque with PD control
        torque = stiffness * q_delta_saturated + damping * q_dot_delta

        return torque