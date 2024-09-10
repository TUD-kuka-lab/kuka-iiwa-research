
"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

import numpy as np
import pickle
import os


subsample_factor = 3
folder = '/demonstrations_post/'
save_folder = '/demonstrations_post_v2/'


# Compute time
def compute_time(delta_t_history):
    time_history = []
    time = 0

    # Sum every delta to get the time of the trajectory
    for delta_t in delta_t_history:
        time = time + delta_t
        time_history.append(time)
        
    return time_history


for filename in os.listdir(folder):
    with open(folder + '/' + filename, 'rb') as file:
        demonstration = pickle.load(file)

    # Prints keys
    print(demonstration.keys()) 

    # Compute time
    delta_t_history = np.array(demonstration['delta_t'])
    time_history = compute_time(delta_t_history)


    # Get joint positions
    q_trajectory = np.array(demonstration['q'])
    n_joints = q_trajectory.shape[1]

    # Get joint velocities
    q_dot_trajectory = np.array(demonstration['q_dot'])
    n_joints = q_dot_trajectory.shape[1]


    # Subsample arrays
    q_trajectory_subsampled = q_trajectory[::subsample_factor]
    q_dot_trajectory_subsampled = q_dot_trajectory[::subsample_factor]
    delta_t_history_subsampled = delta_t_history[::subsample_factor] * subsample_factor
    time_history_subsampled = time_history[::subsample_factor]

    # Compare trajectory lenghts
    original_length = q_trajectory.shape[0]
    subsampled_length = q_trajectory_subsampled.shape[0]

    print('Original trajectory length:', original_length)
    print('Subsampled trajectory length:', subsampled_length)

    # Check new sample frequency mean
    mean_sampling_frequency = 1 / np.array(delta_t_history_subsampled).mean()
    print('Sampling frequency:', mean_sampling_frequency)


    # Create dictionary
    trajectory = {'q': q_trajectory_subsampled,
                  'q_dot': q_dot_trajectory_subsampled,
                  'delta_t': delta_t_history_subsampled,
                  'time': time_history_subsampled}

    # Save dictionary
    newfilepath = save_folder  + '/' + filename

    # check if folder exists
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    with open(newfilepath, 'wb') as file:
        pickle.dump(trajectory, file)
