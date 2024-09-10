"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

from sensor_msgs.msg import JointState
import cor_tud_msgs.msg as cor_msg
import numpy as np
import rospy
import sys

ROBOT = sys.argv[1]


class JointLinearDS:
    def __init__(self):
        self.gain = 1.0
        self.q = None
        rospy.Subscriber('/%s/joint_states' % ROBOT, JointState, self._callback_joint_states, queue_size=10)
        self.request_pub = rospy.Publisher('/%s/control_request' % ROBOT, cor_msg.ControlRequest, queue_size=10)

    def _callback_joint_states(self, data):
        self.q = np.array(data.position)

    def linear_DS(self, q_goal):
        q_delta = self.gain * (q_goal - self.q)
        q_d = self.q + q_delta
        return q_d
    
    def send_request(self, q_d, q_dot_d=[0, 0, 0, 0, 0, 0, 0]):
        msg = cor_msg.ControlRequest()
        msg.header.stamp = rospy.Time.now()
        msg.q_d.data = q_d
        msg.q_dot_d.data = q_dot_d
        msg.control_type = 'joint impedance'
        self.request_pub.publish(msg)
        return True


if __name__ == '__main__':
    # Init ROS
    rospy.init_node('LinearDS')
    control_frequency = 500
    rate = rospy.Rate(control_frequency)
    joint_linear_DS = JointLinearDS()
    rospy.sleep(0.1)
    #q_goal = np.array([0.091, 0.064, -0.096, -1.706, -0.035, -0.199, 0.040])  # cartesian impedance init
    #q_goal = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #q_goal = np.array([-2.226, 1.377, 0.561, -1.559, -0.707, -1.005, -0.857])  # pouring init juice iiwa14
    #q_goal = np.array([-1.881, 1.352, 0.033, -1.279, -0.370, -1.207, -0.379])  # pouring init coke iiwa14
    #q_goal = np.array([-1.876, 1.551, 0.032, -1.112, -0.365, -1.244, -0.3781]) # pouring init coke iiwa7
    q_goal = np.array((-0.002, 0.355, -0.016, -1.212, 0.012, -0.002, -0.010))  # home position task planning
    #q_goal = np.array([0.3345, 0.6105, 0.0942, -1.2751, -0.0452, 1.2371, 0.4159])  # tracking orbitally

    while not rospy.is_shutdown():
        try:
            if joint_linear_DS.q is None:
                continue
            # Get desired state from linear ds
            q_d = joint_linear_DS.linear_DS(q_goal)

            # Send to low-level controller
            joint_linear_DS.send_request(q_d)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
