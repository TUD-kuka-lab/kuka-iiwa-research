"""
Adapted by:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""
import numpy as np
import rospkg
from roboticstoolbox.robot.ERobot import ERobot
from spatialmath import SE3


class iiwa(ERobot):
    """
    Class that imports a KUKA iiwa URDF model

    ``iiwa()`` is a class which imports a KUKA iiwa robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self, model):
        rospack = rospkg.RosPack()
        base_dir = rospack.get_path('cor_tud_controllers')
        
        #print('trying to read: '+ base_dir + '/urdf/%s.urdf.xacro' % model)
        
        #links, name = self.URDF_read(
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            base_dir + '/urdf/%s.urdf.xacro' % model
        )

        super().__init__(
            links, name=name, manufacturer='KUKA', gripper_links=links[9]
        )

        self.grippers[0].tool = SE3(0, 0, -0.044)
        #self.grippers[0].tool = SE3(0.1, 0, 0)

        #self.qdlim = np.array(
         #   [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 3.0, 3.0]
        #)

        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0, 0]))

        #self.addconfiguration("qr", np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4]))


if __name__ == "__main__":
    robot = iiwa('iiwa7')
    print(robot)

    for link in robot.grippers[0].links:
        print(link)
