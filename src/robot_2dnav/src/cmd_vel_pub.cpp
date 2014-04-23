#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class RobotCmdVelPublisher
{
public:
  RobotCmdVelPublisher();

private:
  void Callback(const geometry_msgs::TwistConstPtr& cmd_vel);
  
  ros::NodeHandle node;

  ros::Publisher vel_pub;
  ros::Subscriber vel_sub;
  
};


RobotCmdVelPublisher::RobotCmdVelPublisher()
{

  this->vel_pub = this->node.advertise<geometry_msgs::Twist>("/gazebo/cmd_vel", 1);
  this->vel_sub = this->node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RobotCmdVelPublisher::Callback, this);

}

void RobotCmdVelPublisher::Callback(const geometry_msgs::TwistConstPtr& cmd_vel)
{
  vel_pub.publish(cmd_vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  RobotCmdVelPublisher robot_cmd_vel_pub;

  ros::spin();
}