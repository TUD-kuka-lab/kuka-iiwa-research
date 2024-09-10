"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

import cor_tud_msgs.msg as cor_msg
import rospy
import sys

ROBOT = sys.argv[1]


class GravityCompensation:
    def __init__(self):
        self.request_pub = rospy.Publisher('/%s/control_request' % ROBOT, cor_msg.ControlRequest, queue_size=10)

    def send_position_request(self):
        msg = cor_msg.ControlRequest()
        msg.header.stamp = rospy.Time.now()
        msg.control_type = 'gravity compensation'
        self.request_pub.publish(msg)
        return True


if __name__ == '__main__':
    # Init ROS
    rospy.init_node('GravityCompensation')
    control_frequency = 500
    rate = rospy.Rate(control_frequency)
    gravity_compensation = GravityCompensation()
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            gravity_compensation.send_position_request()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
