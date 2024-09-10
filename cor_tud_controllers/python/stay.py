"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

from sensor_msgs.msg import JointState
import cor_tud_msgs.msg as cor_msg
#from impedance_control.joint_impedance_control import JointImpedanceController
import numpy as np
import rospy
import sys

ROBOT = sys.argv[1]


class Stay:
    def __init__(self):
        self.gain = 1.0
        self.q = None
        self.q_d = None
        rospy.Subscriber('/%s/joint_states' % ROBOT, JointState, self._callback_joint_states, queue_size=10)
        self.request_pub = rospy.Publisher('/%s/control_request' % ROBOT, cor_msg.ControlRequest, queue_size=10)

    def _callback_joint_states(self, data):
        self.q = np.array(data.position)

    def linear_DS(self, q_goal):
        q_delta = self.gain * (q_goal - self.q)
        q_d = self.q + q_delta
        return q_d

    def send_position_request(self):
        if self.q_d is None:
            self.q_d = self.q
        else:
            alpha = 0.001
            self.q_d = (1 - alpha) * self.q_d + alpha * self.q 

        q_d = self.linear_DS(self.q_d)
        msg = cor_msg.ControlRequest()
        msg.header.stamp = rospy.Time.now()
        msg.q_d.data = q_d
        msg.control_type = 'joint impedance'
        self.request_pub.publish(msg)
        return True


if __name__ == '__main__':
    # Init ROS
    rospy.init_node('LinearDS')
    control_frequency = 500
    rate = rospy.Rate(control_frequency)
    joint_linear_DS = Stay()
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            if joint_linear_DS.q is None:
                continue
            joint_linear_DS.send_position_request()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
