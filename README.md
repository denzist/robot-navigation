What was done:
	1)4 wheeled robot simulation - urdf description; gazebo controller //
	
	2)joy teleoperating //
	
	3)odometry calculating //
	
	3)2d navigation demo through face_localization and base_local_controller //
	
//TODO
	Add module which provides service for returning robot into init pose when joy teleoparation turns off//

//HOW TO RUN
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
