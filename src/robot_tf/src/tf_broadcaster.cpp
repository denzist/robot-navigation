#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle nh;

	tf::TransformBroadcaster broadcaster;
	//tf::Transform transform;
	
	double PI = 3.1415926535897931;
	double base_hight = 0.125;
	double base_width = 0.3;
	double base_length = 0.4;

	while(nh.ok()){
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,0.0),
				 tf::Vector3(base_length/2 - base_length/4, 0.0, base_hight/2 + 0.05)),
				ros::Time::now(), "base_link","hokuyo_link"));
	}
}