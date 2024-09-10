"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

import rospy
import time
import pickle
import sys
from sensor_msgs.msg import JointState
from pynput.keyboard import Listener, KeyCode

ROBOT = sys.argv[1]


class StateRecorder:
    def __init__(self, robot_name, save_id):
        self.save_id = save_id
        self.start = False
        self.end = False
        self.q, self.q_dot, self.time_prev = None, None, None
        self.q_history, self.q_dot_history, self.delta_t_history = [], [], []

        # Start keyboard listener
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

        # Create ROS subscriber
        rospy.Subscriber('/%s/joint_states' % robot_name, JointState, self._callback_joint_states, queue_size=10)

    def _callback_joint_states(self, data):
        self.q = data.position
        self.q_dot = data.velocity

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('s'):
            self.start = True
            print('Recording started.')
        elif key == KeyCode.from_char('e'):
            self.end = True
            print('Recording ended.')

    def _get_delta_t(self):
        # Initialize previous time
        if self.time_prev is None:
            self.time_prev = time.time()

        # Get delta t
        delta_t = time.time() - self.time_prev

        # Update previous time
        self.time_prev = time.time()

        return delta_t

    def _append_state(self):
        self.q_history.append(self.q)
        self.q_dot_history.append(self.q_dot)
        self.delta_t_history.append(self._get_delta_t())

    def _save_trajectory(self):
        # Create dictionary
        trajectory = {'q': self.q_history,
                      'q_dot': self.q_dot_history,
                      'delta_t': self.delta_t_history}

        # Save dictionary
        filename = 'joints_state_' + self.save_id + '.pk'
        with open(filename, 'wb') as file:
            pickle.dump(trajectory, file)

    def run(self):
        if self.start:
            # Append state to trajectory
            self._append_state()

        if self.end:
            # Save trajectory and exit
            self._save_trajectory()
            exit()


if __name__ == '__main__':
    rospy.init_node('state_recorder')
    state_recorder = StateRecorder(ROBOT, save_id=sys.argv[2])
    rate = rospy.Rate(100)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            state_recorder.run()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
