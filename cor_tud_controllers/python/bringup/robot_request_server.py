#!/usr/bin/python3

"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
    Zhaoting Li <Z.Li-23@tudelft.nl>
"""
import std_msgs.msg as std_msg
import numpy as np
import rospy
import argparse
import time

# Get arguments
parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', type=str, default='default_robot', help='Name of the robot')
args, _ = parser.parse_known_args()


class RobotRequestServer:
    def __init__(self, control_frequency):
        self.max_torque = np.array([256, 256, 140, 140, 88, 60, 30])
        self.control_frequency = control_frequency
        self.torque_command = None
        robot_name = args.robot_name
        self.command_pub = rospy.Publisher('/%s/TorqueController/command' % robot_name, std_msg.Float64MultiArray, queue_size=10)
        rospy.Subscriber('/%s/server_torque_command' % robot_name, std_msg.Float64MultiArray, self._callback_torque_command, queue_size=10)
        self.last_received_time = None
        self.timeout = 0.5  # timeout in seconds. Adjust as needed

    def _callback_torque_command(self, data):
        self.torque_command = np.array(data.data)
        self.last_received_time = time.time()

    def _is_topic_active(self):
        if self.last_received_time is None:
            return False
        elapsed_time = time.time() - self.last_received_time
        return elapsed_time < self.timeout
    
    def _saturate_torque(self, torque):
        saturated_torque = np.clip(torque, a_min=-self.max_torque, a_max=self.max_torque)
        return saturated_torque

    def _send_control_request(self):
        msg = std_msg.Float64MultiArray()
        if self.torque_command is None:
            msg.data = [0, 0, 0, 0, 0, 0, 0]  # if no request is received, send zero torque
        elif not self._is_topic_active():
            msg.data = [0, 0, 0, 0, 0, 0, 0] 
            rospy.logwarn("No active torque command received in the last {} seconds".format(self.timeout))
        else:
            msg.data = self._saturate_torque(self.torque_command)
        self.command_pub.publish(msg)
        return True

    def run(self):
        rate = rospy.Rate(self.control_frequency)
        rospy.sleep(0.1)
        while not rospy.is_shutdown():
            try:
                # Send torque request to robot
                self._send_control_request()
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == '__main__':
    # Init ROS
    name = 'robot_request_server'
    rospy.init_node(name)
    control_frequency = 1000
    control_server = RobotRequestServer(control_frequency)
    control_server.run()
