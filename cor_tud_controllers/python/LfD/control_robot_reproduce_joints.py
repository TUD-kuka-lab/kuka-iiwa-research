"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""
import rospy
import pickle
import numpy as np
from scipy.interpolate import splprep, splev
from sensor_msgs.msg import JointState
import cor_tud_msgs.msg as cor_msg
from iiwa_robotics_toolbox import iiwa
import sys


class JointControl:
    def __init__(self, demo_dir, robot_name):
        # Control variables and parameters
        self.q = None
        self.spline_joint_control_start = False
        self.position_reach_thr = 0.03
        self.joint_spline_time_factor = 1.0

        # Robotics toolbox
        self.robot = iiwa(model=robot_name)

        # Initialize demonstration
        demonstrations, delta_t = self.load_7D_from_dict(demo_dir)  # load demonstrations of selected primitives
        self.delta_t = delta_t
        self.demos_array = np.array(demonstrations)[:, 0, :]  # reformat demo

        # Get initial/end position
        self.q_initial = self.demos_array[:, 0]
        self.q_end = self.q_initial

        # ROS
        rospy.Subscriber('/%s/joint_states' % robot_name, JointState, self._callback_joint_states, queue_size=10)
        rospy.Subscriber('/%s/end_effector_state' % robot_name, cor_msg.CartesianState, self._callback_end_effector_state, queue_size=10)
        self.request_pub = rospy.Publisher('/%s/control_request' % robot_name, cor_msg.ControlRequest, queue_size=10)

    def _callback_joint_states(self, data):
        self.q = np.array(data.position)

    def _callback_end_effector_state(self, data):
        self.ee_pose = np.array(data.pose.data)
        self.ee_velocity = np.array(data.velocity.data)

    def load_7D_from_dict(self, load_dir):
        q_1, q_2, q_3, q_4, q_5, q_6, q_7 = [], [], [], [], [], [], []

        with open(load_dir, 'rb') as file:
            data = pickle.load(file)
        q = data['q']
        delta_t = np.mean(data['delta_t'])  # assumed to have small variations
        q_1.append(q[:, 0])
        q_2.append(q[:, 1])
        q_3.append(q[:, 2])
        q_4.append(q[:, 3])
        q_5.append(q[:, 4])
        q_6.append(q[:, 5])
        q_7.append(q[:, 6])

        return [q_1, q_2, q_3, q_4, q_5, q_6, q_7], delta_t

    def send_request(self, q_d, q_dot_d=np.zeros(7)):
        msg = cor_msg.ControlRequest()
        msg.header.stamp = rospy.Time.now()
        msg.q_d.data = q_d
        msg.q_dot_d.data = q_dot_d
        msg.control_type = 'joint impedance'
        self.request_pub.publish(msg)
        return True

    def init_spline_joint(self, waypoints_joint):
        # Get time arraw
        recorded_control_frequency = 1 / self.delta_t
        delta_t_sample = 1 / (recorded_control_frequency * self.joint_spline_time_factor)
        epsilon = delta_t_sample * 1e-6
        time_demos = np.arange(delta_t_sample, len(waypoints_joint[:, 0]) * delta_t_sample + epsilon, delta_t_sample)
        self.spline_joint_max_time = np.max(time_demos)

        # Fit spline_cartesian
        self.spline_joint_paremeters, u = splprep([waypoints_joint[:, 0],
                                                   waypoints_joint[:, 1],
                                                   waypoints_joint[:, 2],
                                                   waypoints_joint[:, 3],
                                                   waypoints_joint[:, 4],
                                                   waypoints_joint[:, 5],
                                                   waypoints_joint[:, 6],
                                                   time_demos], s=0)

        spline_data = {'max time': self.spline_joint_max_time,
                       'parameters': self.spline_joint_paremeters}

        return spline_data

    def spline_joint_control(self, position_spline, velocity_spline):
        if not self.spline_joint_control_start:
            self.spline_joint_control_init_time = rospy.Time.now().to_time()
            self.spline_joint_control_start = True

        relative_time = rospy.Time.now().to_time() - self.spline_joint_control_init_time
        u = relative_time / self.spline_joint_max_time
        if u > 1:
            u = 1
            finished = True
        else:
            finished = False

        position_spline_joint_values = np.array(splev(u, position_spline['parameters']))[:7].reshape(7)
        velocity_spline_spline_joint_values = np.array(splev(u, velocity_spline['parameters']))[:7].reshape(7)

        return position_spline_joint_values, velocity_spline_spline_joint_values, finished

    def get_desired_joint_trajectory(self):
        q_d_trajectory = self.demos_array.T[1:]
        q_dot_d_trajectory = (self.demos_array.T[1:] - self.demos_array.T[:-1]) / self.delta_t
        return np.array(q_d_trajectory), np.array(q_dot_d_trajectory)

    def run(self):
        control_frequency = 500
        rate = rospy.Rate(control_frequency)
        rospy.sleep(0.1)

        # Run state machine
        state = 'init_spline'
        while not rospy.is_shutdown():
            try:
                # Do nothing until q is received
                if self.q is None:
                    continue

                if state == 'init_position':
                    # Send initial position to low level controller
                    self.send_request(self.q_initial, self.q_initial*0)
                    print(np.linalg.norm(self.q[:6] - self.q_initial[:6]))
                    if np.linalg.norm(self.q[:6] - self.q_initial[:6]) < self.position_reach_thr:
                        print('Position initialized!')

                        # Update state
                        state = 'draw'

                if state == 'init_spline':
                    # Get trajectory
                    q_d_trajectory,  q_dot_d_trajectory = self.get_desired_joint_trajectory()

                    # Init spline
                    position_spline = self.init_spline_joint(q_d_trajectory)
                    velocity_spline = self.init_spline_joint(q_dot_d_trajectory)

                    # Update state
                    state = 'init_position'

                if state == 'draw':
                    # Get desired position from CONDOR
                    q_d, q_dot_d, finished = self.spline_joint_control(position_spline, velocity_spline)

                    # Send position to low level controller
                    self.send_request(q_d, q_dot_d)

                    if finished:
                        return True

                # Sleep
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == '__main__':
    # Init ROS
    rospy.init_node('joint_reproduction')

    # Run draw demo
    demo_dir = 'demonstrations_post/post_joints_state_1.pk'
    robot_name = sys.argv[1]
    CONDOR_control = JointControl(demo_dir, robot_name)
    CONDOR_control.run()
