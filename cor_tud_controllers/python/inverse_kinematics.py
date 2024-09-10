from iiwa_robotics_toolbox import iiwa
from spatialmath import SE3, UnitQuaternion
from trac_ik_python.trac_ik import IK
import numpy as np
import rospy
import ctypes
import rospkg
import sys
import os
import yaml

path_to_relaxed_ik = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'
sys.path.insert(1, path_to_relaxed_ik + '/wrappers')
from python_wrapper import RelaxedIKRust


def inverse_kinematics_init(type, q=None, robot_name='iiwa7'):
    if type == 'robotics toolbox':
        return IK_robotic_toolbox(q, robot_name)
    elif type == 'track ik':
        return IK_track_ik(q, robot_name)
    elif type == 'ranged ik':
        return RangedIK(robot_name)


class IK_robotic_toolbox:
    def __init__(self, q, robot_name):
        self.robot = iiwa(model=robot_name)

    def compute(self, position_d, orientation_d, q):
        # Get space transform
        se3 = np.identity(4)
        se3[:3, :3] = orientation_d.A
        se3[:3, 3] = position_d
        T = SE3(se3)


        # Compute inverse kinematics, other options from the robotics toolbox could be employed in this step
        q_d = self.robot.ikine_LMS(T, q)[0]

        return q_d


class IK_track_ik:
    def __init__(self, q, robot_name):
        urdf_str = rospy.get_param('/%s_description' % robot_name)
        self.trac_ik_solver = IK('iiwa_link_0', 'iiwa_link_7', urdf_string=urdf_str, solve_type='Distance')

    def compute(self, position_d, orientation_d, q):
        # Map radians in euler to quaternions
        orientation_d_quat_rev = UnitQuaternion.RPY(orientation_d).A

        # Change order quaternion (mismatch between libraries)
        orientation_d_quat = np.zeros(4)
        orientation_d_quat[:3] = orientation_d_quat_rev[1:]
        orientation_d_quat[3] = orientation_d_quat_rev[0]

        # Get ik solution
        q_d = self.trac_ik_solver.get_ik(q, position_d[0], position_d[1], position_d[2], orientation_d_quat[0],
                                         orientation_d_quat[1], orientation_d_quat[2], orientation_d_quat[3])

        # Check if no solution was found
        if q_d is None:
            q_d = np.array(q)
            print('IK: no solution found! \n')

        return np.array(q_d)
    

class RangedIK():
    def __init__(self, robot_name):
        path_cor_controllers = rospkg.RosPack().get_path('cor_tud_controllers')

        # Set path
        setting_file_path = path_cor_controllers + ('/python/settings_ranged_ik/settings_%s.yaml' % robot_name) 
        #setting_file_path = path_to_src + ('/configs/settings_%s.yaml' % robot_name) 
        os.chdir(path_to_relaxed_ik)

        # Load the infomation
        setting_file = open(setting_file_path, 'r')
        settings = yaml.load(setting_file, Loader=yaml.FullLoader)
       
        urdf_file = open(path_to_relaxed_ik + '/configs/urdfs/' + settings["urdf"], 'r')
        urdf_string = urdf_file.read()
        rospy.set_param('robot_description', urdf_string)

        # Initialize IK
        self.relaxed_ik = RelaxedIKRust(setting_file_path)

        print("\nSolver RelaxedIK initialized!\n")

    def compute(self, position_d, orientation_d, q):
        # Reset state ranged ik
        n = 7  # dimensionality of C space
        q_ranged_ik = (ctypes.c_double * n)()
        for i in range(n):
            q_ranged_ik[i] = q[i]
        self.relaxed_ik.reset(q_ranged_ik)

        # Map SO3 to unit quaternions
        orientation_d_quat = UnitQuaternion.RPY(orientation_d).A

        # Solve
        positions = []
        orientations = []
        tolerances = []

        positions.append(position_d[0])
        positions.append(position_d[1])
        positions.append(position_d[2])
        orientations.append(orientation_d_quat[1])
        orientations.append(orientation_d_quat[2])
        orientations.append(orientation_d_quat[3])
        orientations.append(orientation_d_quat[0])
        for _ in range(3):
            tolerances.append(0.0)
        for _ in range(3):
            tolerances.append(0.075)

        ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        return np.array(ik_solution)
