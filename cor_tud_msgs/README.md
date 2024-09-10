### Heading:
Authors:
- Leandro de Souza Rosa <l.desouzarosa@tudelft.nl>

Tested on:
- **Ubuntu 20.04, ROS noetic** - Leandro

# CoR messages  
This `ROS` module contains definitions of `message`s, `service`s, and `action`s used in the group's applications for the KUKA's robots.

### Message
1. -

### Service
1. -

### Action
Note that these files only define the interface. The actual implementation is up to the user. The descriptions bellow are just an indication on how the interfaces are used.

1. [SetJointPose](/src/cor_tud_msgs/action/SetJointPose.action)
   - Goal: 
     - `Float64[] pose` - joint poses for `PositionController`.
     - `Float64 precision` - Error between requested pose and readings from sensors (`/iiwa/joint_states`).
     - 'float64 rate` - update rate in Hz.
   - Result: `bool success` - True is the achieved pose is within the `precision`.
   - Feedback: `bool finished` - False while the controler is moving to the required pose. True after achieving it.

2. [RecordTopic](/src/cor_tud_msgs/action/RecordTopic.action)
   - Goal:
     - `bool start` - start or stop the recording `True/False`.
     - `string topic` - topic to record from.
     - `string file` - file to record on.
   - Result: `bool success` - True when the record is finished. 
   - Feedback: `bool finished` - True when the record is finished.

3. [Controller](/src/cor_tud_msgs/action/Controller.action)
   - Goal:
     - `float64[] reference` - position for joints, end-effector, etc.
     - `float64[] stiffness` - stiffness of the joints, end-effector, etc.
     - `string mode` -  Control mode, 'cartesian', 'joint', etc... see implemented modes in the controller [server](/src/cartesian_impedance_controller/python/torque_controller.py).
     - `float64 time` - Time budget to complete action.
     - `float64 precision - Minimum precision w.r.t. the reference to complete action.
     - `float64 - rate` - Update rate.
   - Result: `bool success` - True when the record is finished.     
   - Feedback: `bool finished` - True when the record is finished.
