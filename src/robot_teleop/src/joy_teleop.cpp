#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>


class RobotJoyTeleop
{
public:
  RobotJoyTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle node;

  double max_vel_x, max_rotational_vel;
  double speed_multiplier;
  std::string cmd_topic;

  ros::Publisher vel_pub;
  ros::Publisher verification_pub;
  ros::Subscriber joy_sub;
  
};


RobotJoyTeleop::RobotJoyTeleop()
{
  node.param("/max_lin_vel", this->max_vel_x, 1.3);
  node.param("/max_ang_vel", this->max_rotational_vel, 1.0);
  node.param<std::string>("/cmd_topic", this->cmd_topic, "/gazebo/cmd_vel");
  if (node.hasParam("/cmd_topic"))
  {
    ROS_INFO("No param named 'cmd_topic'");
  }
  if (node.hasParam("/max_lin_vel"))
  {
    ROS_INFO("No param named 'max_lin_vel'");
  }
  if (node.hasParam("/max_ang_vel"))
  {
    ROS_INFO("No param named 'max_ang_vel'");
  }

  this->vel_pub = this->node.advertise<geometry_msgs::Twist>(this->cmd_topic, 1);

  this->joy_sub = this->node.subscribe<sensor_msgs::Joy>("joy", 10, &RobotJoyTeleop::joyCallback, this);

  this->verification_pub = this->node.advertise<std_msgs::Int32>("/teleop_verification", 1);

}

void RobotJoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  speed_multiplier = 1.0;
  // check if cross is used for steering
  vel.linear.x = max_vel_x * joy->axes[1] * speed_multiplier;
  vel.angular.z = max_rotational_vel * joy->axes[0] * speed_multiplier;
  vel_pub.publish(vel);

  std_msgs::Int32 msg;
  msg.data = 1;
  this->verification_pub.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  RobotJoyTeleop robot_joy_teleop;

  ros::spin();
}

