#include "autonomous_return.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "autonomous_robot_return_system");
	ros::NodeHandle node("~");
	ros::Rate rate(100.0);
	AutonomousReturnSystem arrs;
  	while(node.ok()){
    	arrs.spinOnce();
    	rate.sleep();
  	}
  	node.shutdown();
  	return 0;
}