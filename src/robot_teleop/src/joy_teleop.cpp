#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>


class RobotJoyTeleop
{
public:
  RobotJoyTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle node;

  double max_vel_x, max_rotational_vel;
  double speed_multiplier;

  ros::Publisher vel_pub;
  ros::Subscriber joy_sub;
  
};


RobotJoyTeleop::RobotJoyTeleop()
{

  this->node.param("max_vel_x", this->max_vel_x,1.0);
  this->node.param("max_rotational_vel", this->max_rotational_vel,5.0);


  this->vel_pub = this->node.advertise<geometry_msgs::Twist>("/gazebo/cmd_vel", 1);

  this->joy_sub = this->node.subscribe<sensor_msgs::Joy>("joy", 10, &RobotJoyTeleop::joyCallback, this);

}

void RobotJoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;

  if (joy->buttons[4] == 1)   // check for full-speed button
  {
    speed_multiplier = 1.0;
  }
  else  // check if right analog stick was used to scale speed
  {
    speed_multiplier = 0.5 + (0.5 * joy->axes[4]);  // stick full front -> speed_multiplier = 1.0 , full back -> 0.0
  }

  // check if cross is used for steering
    vel.linear.x = max_vel_x * joy->axes[1] * speed_multiplier;
    vel.angular.z = max_rotational_vel * joy->axes[0] * speed_multiplier;
  vel_pub.publish(vel);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  RobotJoyTeleop robot_joy_teleop;

  ros::spin();
}

