import pickle
from os import listdir, makedirs
import sys
from os.path import isfile, join, dirname, realpath, abspath, exists
import numpy as np
SCRIPT_DIR = dirname(abspath(__file__))
sys.path.append(dirname(SCRIPT_DIR))  # add parent directory to path
from iiwa_robotics_toolbox import iiwa

# Get robot from robotics toolbox
robot = iiwa(sys.argv[1])

# Get directory with demonstrations
directory_file = dirname(realpath(__file__))
directory_demonstrations = directory_file + '/demonstrations_post/'

# Get file names of demonstrations
demonstrations_file_names = [f for f in listdir(directory_demonstrations) if isfile(join(directory_demonstrations, f))]

# Iterate through every demonstration
demonstration_id = 0
for demonstration_file_name in demonstrations_file_names:

    # Load demonstrations
    demonstration_directory = directory_demonstrations + demonstration_file_name
    print(demonstration_file_name)
    with open(demonstration_directory, 'rb') as file:
        demonstration = pickle.load(file)


    # Create new dictionary with ee data
    ee_demonstration = {'x_pos': [],
                        'x_rot': [],
                        'x_dot': [],
                        'delta_t': demonstration['delta_t']}

    # Iterate through every point in the demonstration
    for point_i in range(len(demonstration['q'])):
        # Get robot end effector pose
        ee_pose_SE3 = robot.fkine(demonstration['q'][point_i], end='iiwa_link_7', start='iiwa_link_0')
        ee_position = ee_pose_SE3.t
        ee_orentation = ee_pose_SE3.R

        # Get robot end effector velocity
        ee_velocity = np.matmul(robot.jacob0(demonstration['q'][point_i], end='iiwa_link_7', start='iiwa_link_0'), demonstration['q_dot'][point_i])

        # Save in demonstration dictionary
        ee_demonstration['x_pos'].append(ee_position)
        ee_demonstration['x_rot'].append(ee_orentation)
        ee_demonstration['x_dot'].append(ee_velocity)

    # Save demonstration end effector
    ee_directory_demonstrations = directory_file + '/ee_demonstrations/'

    # Create directory if it does not exist
    if not exists(ee_directory_demonstrations):
        makedirs(ee_directory_demonstrations)

    # Save demonstrations
    ee_demonstration_directory = ee_directory_demonstrations + 'ee_state_' + demonstration_file_name[len('joint_state_'):-3] + '.pk'
    demonstration_id += 1
    with open(ee_demonstration_directory, 'wb') as file:
        pickle.dump(ee_demonstration, file)

