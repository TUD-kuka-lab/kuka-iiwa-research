"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

from sensor_msgs.msg import JointState
from iiwa_robotics_toolbox import iiwa
from low_level_controllers.joint_impedance_control import JointImpedanceController
from low_level_controllers.cartesian_impedance_control import CartesianImpedanceController
from inverse_kinematics import inverse_kinematics_init
from spatialmath import SO3
import cor_tud_msgs.msg as cor_msg
import sensor_msgs.msg as sensor_msg
import numpy as np
import rospy
import sys

ROBOT = sys.argv[1]

class Spacenav:
    def __init__(self):
        self.joint_impedance_controller = JointImpedanceController(robot_name=ROBOT, alpha=0.5)
        self.robot = iiwa(model=ROBOT)
        self.cartesian_impedance_controller = CartesianImpedanceController(robot_name=ROBOT, robot=self.robot)
        self.IK = None
        
        # ROS
        rospy.Subscriber('/spacenav/joy', sensor_msg.Joy, self._callback_spacenav, queue_size=10)
        rospy.Subscriber('/%s/end_effector_state' % ROBOT, cor_msg.CartesianState, self._callback_end_effector_state, queue_size=10)
        rospy.Subscriber('/%s/joint_states' % ROBOT, JointState, self._callback_joint_states, queue_size=10)
        
        # Init variables
        self.q = None
        self.ee_pose = None
        self.spacenav_state = None
        
        # Parameters
        self.controller = 'cartesian_impedance'  # options: cartesian_impedance, ik
        self.control_orientation = True
        self.scale = 0.1
        self.delta_q_max = np.array([0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08])  # maximum requested delta for safety

        # If self.control_orientation = False
        #self.orientation_goal = np.array([0, 90, 0])
        self.orientation_goal = np.array([0, 0, 0])
        self.delta_t_orientation = 1
        self.gain_orientation = 0.1
        
        
    def _callback_spacenav(self, data):
        self.spacenav_state = data.axes
        if data.buttons[0] and data.buttons[1]:
            self.store_data = True

    def _callback_end_effector_state(self, data):
        self.ee_pose = np.array(data.pose.data)
        
    def _callback_joint_states(self, data):
        self.q = np.array(data.position)

    def init_IK(self):
        self.IK = inverse_kinematics_init('track ik', self.q, ROBOT)

    def _linear_DS_orientation(self, orientation_d, orientation_matrix):
        orientation_d_matrix = SO3.RPY(orientation_d, order='zyx')
        orientation_dot_d = (orientation_d_matrix / orientation_matrix)
        return orientation_dot_d

    def run(self):
        # Get ee pose
        ee_position, ee_orientation = self.ee_pose[:3], SO3.RPY(self.ee_pose[3:], order='zyx')

        # Get delta x
        ee_delta_position = np.array(self.spacenav_state)[:3]

        # Get desired position
        ee_goal_position = ee_position + ee_delta_position * self.scale
        
        if self.control_orientation:
            ee_delta_orientation = np.array(self.spacenav_state)[3:] * self.scale
            ee_delta_orientation = SO3.RPY(ee_delta_orientation, order='zyx')
        else:
            orientation_d_dot = self._linear_DS_orientation(self.orientation_goal, ee_orientation)
            ee_delta_orientation =  orientation_d_dot * self.delta_t_orientation

        # Get desired orientation
        ee_goal_orientation = ee_delta_orientation @ ee_orientation

        if self.controller == 'cartesian_impedance':
            # Send command to controller
            self.cartesian_impedance_controller.control_law(ee_goal_position, ee_goal_orientation)

        elif self.controller == 'ik':
            # Compute inverse kinematics
            q_d = self.IK.compute(ee_goal_position, ee_goal_orientation.rpy(), self.q)

            # Compute delta in joint space and store
            delta_q = q_d - self.q

            # Clip delta
            delta_q_clipped = np.clip(delta_q, a_min=-self.delta_q_max, a_max=self.delta_q_max)

            # Compute desired joint clipped
            q_d_clipped = self.q + delta_q_clipped

            # Send command to controller
            self.joint_impedance_controller.send_torque_request(q_d_clipped)
        else:
            raise ValueError('Selected controller not valid, options: cartesian_impedance, ik.')

        return True
            
if __name__ == '__main__':
    rospy.init_node('robot_control')
    control_frequency = 500
    rate = rospy.Rate(control_frequency)
    control = Spacenav()
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            if control.q is None:
                continue

            # Start IK with current q
            control.init_IK()

            # Run control
            control.run()

        except rospy.ROSInterruptException:
            pass

        rate.sleep()

