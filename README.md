TUD CoR fork from https://github.com/epfl-lasa/iiwa_ros

Controllers, codes and examples to work with the KUKAs

Installation Instructions
 
ROS Stack for KUKA's IIWA robots
---------------------------------------
 
  - Use the Fast Research Interface (FRI) to connect to the robot
  - Integration with ROS control
  - Gazebo integration with gravity compensation (similar to real robot)

Requirements
-----------

iiwa_ros requires several packages to be installed in order to work properly:

* [ROS] - ROS: tested in **Melodic** and **Kinetic**; *Indigo* should work also. **Noetic** is working.
* [KUKA FRI] - This is a modified version of the C++ FRI library provided by KUKA. ***DO NOT SHARE THIS CODE WITHOUT EXPLICIT PERMISSION FROM YOUR RESPOSIBLE!!!***
* [ROS Control]
* [Gazebo] and gazebo-ros-pkgs
* [SpaceVecAlg]
* [RBDyn]
* [mc_rbdyn_urdf]
* [corrade]
* [robot_controllers]
* [robotics-toolbox] Required version 0.11.0, Requires Python >= 3.6.

Setting Up
-------------
In the next steps you will compile and install a series of libraries. The installation's will write the files in your system's standard folders, so it is a good practice keep them close to this repo in case you need to clean up.

TODO: Implement instructions to install the dependencies on non standard folders

```sh
mkdir <home_folder>
cd <home_foder>
```

Double check your gcc and g++ versions
* [Ubuntu 20.04] - Tested with ROS Noetic, gcc,g++ 10. Fails with gcc,g++ 9. Python3.8.
* [Ubuntu 18.04] - Tested with ROS Melodic, gcc 7.5.

Installing Dependencies
-------------

### KUKA FRI (Private)
***DO NOT SHARE THIS CODE WITHOUT EXPLICIT PERMISSION FROM YOUR RESPOSIBLE!!!***

```sh
cd <home_folder>
git clone https://gitlab.tudelft.nl/kuka-iiwa-7-cor-lab/kuka_fri.git
cd kuka_fri
git checkout legacy
./waf configure
./waf
sudo ./waf install
```

### SpaceVecAlg

```sh
cd <home_folder>
git clone --recursive https://github.com/costashatz/SpaceVecAlg.git
cd SpaceVecAlg
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### RBDyn

```sh
cd <home_folder>
git clone --recursive https://github.com/costashatz/RBDyn.git
cd RBDyn
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### mc_rbdyn_urdf

```sh
cd <home_folder>
git clone --recursive https://github.com/costashatz/mc_rbdyn_urdf.git
cd mc_rbdyn_urdf
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### corrade

We are using a specific version of Corrade.

```sh
cd <home_folder>
git clone https://github.com/mosra/corrade.git
cd corrade
git checkout 0d149ee9f26a6e35c30b1b44f281b272397842f5
mkdir build && cd build
cmake ..
make -j
sudo make install
```

### robot_controllers

```sh
cd <home_folder>
git clone https://github.com/epfl-lasa/robot_controllers.git
cd robot_controllers
mkdir build && cd build
cmake ..
make -j
sudo make install
```

Compilation
------------

```sh
git clone git@gitlab.tudelft.nl:kuka-iiwa-7-cor-lab/iiwa_ros.git
cd iiwa_ros
source <ROS_HOME>/<dist>/setup.<shell>
catkin_make
```

To use the modules, do not forget to export the current environment:
```sh
source devel/setup.<shell>
```

Setting Up Commands
------------
We recommend to set up commands to source `ROS` and `iiwa_ros`. You can copy+paste the following example commands to your `~/.<shell>rc` file, and edit `<ros dist>` with whatever you want, `<ROS_HOME>/<dist>` with the path and distribution of your favourite `ROS` installation, and `<home_folder>` with the chosen one for this installation (where you cloned this repo).

```bash
alias <ros dist>="<ROS_HOME>/<dist>/setup.<shell>"
alias iiwa="source <home_folder>/iiwa_ros/devel/setup.<shell>"
```

Usage:
 - *`<ros dist>`:* sources the ros distribution.
 - *`iiwa`*: will source the iiwa stack.

Posterior Dependencies
------------

### robotics-toolbox

1. Install the robotics-toolbox for python following the instructions at their [github page](https://github.com/petercorke/robotics-toolbox-python#installing).

   - Installing with `pip` and no flags (yet so fat)

2. Create a link for the robot description files in the `robotics-toolbox` folder. 

```sh
sudo mkdir $(python -m site --user-site)/rtbdata/xacro/iiwa_description
sudo ln -s ~/repos/iiwa/iiwa_ros/src/iiwa_description/meshes $(python -m site --user-site)/rtbdata/xacro/iiwa_description/meshes
```

### ROS packages

You might have to install a fill ros packages to run some applications. ***Packages may vary according to your ROS release***:
```sh
sudo apt install ros-noetic-joint-trajectory-controller
sudo apt install ros-noetic-moveit-simple-controller-manager
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers
```

### Ranged IK
Clone `relaxed_ik_ros1` repository (tested with commit: `7def3bd`, branch: `ranged-ik`)
```sh
git clone https://github.com/uwgraphics/relaxed_ik_ros1.git
```
Include submodule `relaxed_ik_core` (tested with commit: `d58c494`, branch: `ranged-ik`):
```sh
git submodule update --init --recursive
```
Go to folder `relaxed_ik_core` and install:
```sh
cargo build
```

Copy files `matlab_iiwa7.urdf` and `matlab_iiwa14.urdf` and paste them in the folder `<relaxed ik core dir>/relaxed_ik_core/configs/urdfs/`.

Remember to build ROS workspace:
```sh
catkin build
```

Starting the controller
-----------
Set up ROS in the remote computer (also works for local if you are using the simulator):

```sh
export ROS_MASTER_URI=http://localhost:30202
export ROS_HOSTNAME=localhost
```

Set up ROS in the local computer:

```sh
export ROS_MASTER_URI=http://192.180.1.5:30202
export ROS_IP=192.180.1.<YOUR_IP>
```

<robot_model> depends on the robot that you are using, options: 7, 14.

Launch ROS remote:
```sh
roslaunch cor_tud_controllers bringup_remote.launch model:=<robot_model> simulation:=<true/false>
```

Launch ROS remote:
```sh
roslaunch cor_tud_controllers bringup_local.launch model:=<robot_model> 
```


Misc
--------------

If your system has problems to find `catkin_make`
```
ImportError: "from catkin_pkg.package import parse_package" failed: No module named 'catkin_pkg'
Make sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.
```

Use `dpkg -L python-catkin-pkg` to find its location, and `catkin_make -DPYTHON_EXECUTABLE=<path to python's catkin>` for compiling.

To use the modules, do not forget to export the current environment:
```sh
source devel/setup.<shell>
```


Credits
--------------
- Rodrigo Pérez Dattari
- Leandro de Souza Rosa
- Micah Prendergast
- Nicky Mol
- Zhaoting Li
- Saray Bakker
- LASA @ https://github.com/epfl-lasa/iiwa_ros
