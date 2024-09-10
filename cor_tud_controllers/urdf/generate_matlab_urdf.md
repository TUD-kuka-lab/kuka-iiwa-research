Source ROS and source IIWA
run the following commands to generate .urdf files which can be loaded with MATLAB's [Robotics System Toolbox](https://nl.mathworks.com/products/robotics.html)


```sh
rosrun xacro xacro -o ./matlab_iiwa7.urdf ./iiwa7.urdf.xacro
rosrun xacro xacro -o ./matlab_iiwa14.urdf ./iiwa14.urdf.xacro
```
