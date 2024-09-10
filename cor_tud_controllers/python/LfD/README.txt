Demo that reproduces joint space demonstration

1) Collect demonstrations: python record_demo_joint_state.py <robot_name> <id>

2) Create folder: joint_state_demos

3) Postprocess demonstrations:
	i) jupyter notebook
	ii) Open and execute (postprocessa according to your preferences): demonstrations_posprocessing.ipynb
	
4) Run controller:
	i) Terminal 1, start rosnode: roslaunch cor_tud_controllers bringup.launch robot_name:=iiwa model:=14 controller:=TorqueController
	ii) Terminal 2, start remote impedance controller: 
		a) go to folder in ROS workspace iiwa_ros/src/cor_tud_controllers/python/impedance_control
		b) run: python joint_impedance_control_remote.py
	iii) Terminal 3, run controller:
		a) go to folder in ROS workspace src/iiwa_ros/src/cor_tud_controllers/python/LfD
		b) run: python control_robot_reproduce_joints.py
		
This code was tested with the iiwa14 and running the java program FRIOverlayMultiTool500. 
