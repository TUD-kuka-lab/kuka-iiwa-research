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


class RequestCartesian:
    def __init__(self):
        self.q = None
        rospy.Subscriber('/%s/joint_states' % ROBOT, JointState, self._callback_joint_states, queue_size=10)
        self.request_pub = rospy.Publisher('/%s/control_request' % ROBOT, cor_msg.ControlRequest, queue_size=10)
        self.control_type = 'cartesian ik'  # options: 'cartesian ik', 'cartesian impedance'

    def _callback_joint_states(self, data):
        self.q = np.array(data.position)
    
    def send_request(self, ee_position_goal, ee_orientation_goal):
        msg = cor_msg.ControlRequest()
        msg.header.stamp = rospy.Time.now()
        msg.pose_d.data = list(ee_position_goal) + list(ee_orientation_goal)
        msg.control_type = self.control_type
        self.request_pub.publish(msg)
        return True


if __name__ == '__main__':
    # ee_position_goal = np.array([0.5, 0, 0.7])
    # ee_orientation_goal = np.array([0, 90, 0]) * np.pi / 180
    # tomato 1
    ee_position_goal = np.array([0.737, 0.163, 0.461])
    #ee_position_goal = np.array([0.737, 0.163, 0.661])
    ee_orientation_goal = np.array([-1.603, 0.123, -1.628])
    control_frequency = 500

    # Init ROS
    rospy.init_node('LinearDS')
    rate = rospy.Rate(control_frequency)

    # Init Controller
    request_cartesian = RequestCartesian()
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            if request_cartesian.q is None:
                continue
            request_cartesian.send_request(ee_position_goal, ee_orientation_goal)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
