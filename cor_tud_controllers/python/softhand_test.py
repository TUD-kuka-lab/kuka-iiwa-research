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
        self.scale = 0.5
        self.close_ratio = 0.7

        # Soft hand publisher
        self.command = rospy.Publisher('/qbhand1/control/qbhand1_synergy_trajectory_controller/command', JointTrajectory, queue_size=10)

    def close(self, t):
        point = JointTrajectoryPoint()
        if self.scale * t < 1:  # if goal hasn't been reached
            point.positions = [t * self.scale * self.close_ratio]
        else:
            point.positions = [self.close_ratio]

        point.time_from_start.nsecs = int(0.7 * 1e9)  # seconds to nanoseconds
        msg = JointTrajectory()
        msg.joint_names = ['qbhand1_synergy_joint']
        msg.points = [point]
        self.command.publish(msg)

    def open(self, t):
        point = JointTrajectoryPoint()
        if self.scale * t < 1:  # if goal hasn't been reached
            point.positions = [1 - t * self.scale]
        else:
            point.positions = [0.0]

        point.time_from_start.nsecs = int(0.7 * 1e9)  # seconds to nanoseconds
        msg = JointTrajectory()
        msg.joint_names = ['qbhand1_synergy_joint']
        msg.points = [point]
        self.command.publish(msg)

    def control(self, t, type):
        point = JointTrajectoryPoint()
        if self.scale * t < 1:  # if goal hasn't been reached
            if type == 'open':
                point.positions = [1 - t * self.scale]
            elif type == 'close':
                point.positions = [t * self.scale * self.close_ratio]
        else:
            if type == 'open':
                point.positions = [0.0]
            if type == 'close':
                point.positions = [self.close_ratio]

        point.time_from_start.nsecs = int(0.7 * 1e9)  # seconds to nanoseconds
        msg = JointTrajectory()
        msg.joint_names = ['qbhand1_synergy_joint']
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
    rate = rospy.Rate(100)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            soft_hand.run(type)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
