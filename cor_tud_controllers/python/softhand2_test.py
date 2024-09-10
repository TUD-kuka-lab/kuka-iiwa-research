"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import time
import sys


class SoftHandControl:
    def __init__(self):
        self.time_init = None
        self.scale = 0.3
        self.manipulation_joint_reference = 0.4
        self.synergy_joint_reference = 1.0

        # Soft hand publisher
        self.command = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=10)

    def control(self, t, type):
        point = JointTrajectoryPoint()
        if self.scale * t < 1:  # if goal hasn't been reached
            if type == 'open':
                point.positions = [self.manipulation_joint_reference, self.synergy_joint_reference - t * self.scale]
            elif type == 'close':
                point.positions = [self.manipulation_joint_reference, t * self.scale]
        else:
            if type == 'open':
                point.positions = [self.manipulation_joint_reference, 0.0]  # only synergy reference is changed for opening
            if type == 'close':
                point.positions = [self.manipulation_joint_reference, self.synergy_joint_reference]

        point.time_from_start.nsecs = int(0.7 * 1e9)  # seconds to nanoseconds
        msg = JointTrajectory()
        msg.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
        msg.points = [point]
        self.command.publish(msg)

    def run(self, type):
        if self.time_init is None:
            self.time_init = time.time()

        t = time.time() - self.time_init
        self.control(t, type=type)


if __name__ == '__main__':
    rospy.init_node('soft_hand_control')
    type = sys.argv[1]
    soft_hand = SoftHandControl()
    rate = rospy.Rate(500)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            soft_hand.run(type)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass