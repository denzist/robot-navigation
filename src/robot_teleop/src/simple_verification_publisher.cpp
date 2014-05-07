#include "ros/ros.h"
#include "std_msgs/Int32.h"



int main(int argc, char **argv)
{

  ros::init(argc, argv, "teleop_veification");
  ros::NodeHandle n;
  ros::Publisher verify_pub = n.advertise<std_msgs::Int32>("/teleop_veification", 1000);
  while (ros::ok())
  {
    std_msgs::Int32 msg;
    msg.data = 1;
    verify_pub.publish(msg);
    ros::spinOnce();
  }
  return 0;
}