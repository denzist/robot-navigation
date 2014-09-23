#include "rtc_tiny_teleop/com.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

//TODO find more appropriate model for effort control
//find out smth about ros_control

enum driver { RIGHT, LEFT };
const short effort_lin_k = 10;
const short effort_ang_k = 100;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rtc_tiny_effort_controller");
	RTK_Tiny rtc_tiny_effort_controller;
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(10);
	boost::shared_ptr<geometry_msgs::Twist const> vel_ptr;
	short effort_commands[2];
	while(ros::ok())
	{
		effort_commands[RIGHT] = 0;
		effort_commands[LEFT] = 0;

		vel_ptr = ros::topic::waitForMessage<geometry_msgs::Twist>(
			"/cmd_vel", ros::Duration(1.0));
		if(vel_ptr != NULL)
		{
			double lin_vel = (*vel_ptr).linear.x;
			double ang_vel = (*vel_ptr).angular.z;
			effort_commands[RIGHT] += (short)(lin_vel * effort_lin_k);
			effort_commands[LEFT] += (short)(lin_vel * effort_lin_k);
			effort_commands[RIGHT] += (short)(-1 * ang_vel * effort_ang_k);
			effort_commands[LEFT] += (short)(ang_vel * effort_ang_k);
		}
		ROS_INFO_STREAM("effort commands right = " << effort_commands[RIGHT] << 
			" left = " << effort_commands[LEFT]);
		rtc_tiny_effort_controller.move(effort_commands[RIGHT], effort_commands[LEFT]);
		loop_rate.sleep();
	}
	return 0;
}
