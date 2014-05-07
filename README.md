What was done:

	1)4 wheeled robot simulation - urdf description; gazebo controller //
	
	2)joy teleoperating //
	
	3)odometry calculating //
	
	3)2d navigation demo through face_localization and base_local_controller //
	
	//NEW
	4)added emergency move base controller
	
	
//HOW TO RUN

	To run emergency move base controller
	
	(with joy teleop)
	
	$ roslaunch robot_gazebo willow_garage_teleop.launch
	$ roslaunch robot_description 2dnav.launch
	$ roslaunch robot_2dnav emergency_move_base.launch

	(with keyboard teleop)
	$ roslaunch robot_gazebo willow_garage.launch
	$ roslaunch robot_teleop keyboard_teleop.launch
	$ roslaunch robot_description 2dnav.launch
	$ roslaunch robot_2dnav emergency_move_base.launch
	
	How it works.
	If you turned off your joy(just pull it out USB)  or keyboard_teleop(to turn it off press ctrl+C twice)
	move base would go into emergency mode and will move to init point. If you turned on joy and keyboard_teleop again,
	robot would be under teleop control. 
	
	To run demo make following commands
	
	**if updated project
	
	1)delete build and devel diractories
	
	in bash
	
	$ cd $(prokect_name)
	$ catkin_make
	
	reopen the bash
	
	** for 2dnav
	
	$ roslaunch robot_gazebo willow_garage.launch 
	$ roslaunch robot_description 2dnav.launch
	$ roslaunch robot_2dnav move_base_demo.launch
	
	** in rviz use button *navigation goal* to send the goal
	
	** for joy teleoparating
	
	$ roslaunch robot_gazebo willow_garage_teleop.launch
	
	or
	
	$ roslaunch robot_gazebo empty_world_teleop.launch
	
	** keyboard teleoparation
	
	$ roslaunch robot_gazebo willow_garage.launch
	$ roslaunch robot_teleop keyboard_teleop.launch
	
	or
	
	$ roslaunch robot_gazebo empty_world_teleop.launch
	$ roslaunch robot_teleop keyboard_teleop.launch
	
