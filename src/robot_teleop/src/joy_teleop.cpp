#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>


class RobotJoyTeleop
{
public:
  RobotJoyTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle* nh_;

  double max_vel_x_;
  double max_rotational_vel_;
  double speed_multiplier_;
  std::string cmd_topic_;

  ros::Publisher vel_pub_;
  ros::Publisher verification_pub_;
  ros::Subscriber joy_sub_;
};


RobotJoyTeleop::RobotJoyTeleop():
  max_vel_x_(1.3),
  max_rotational_vel_(1.0),
  cmd_topic_("/gazebo/cmd_vel")
{
  nh_ = new ros::NodeHandle("~");
  nh_->param("/max_lin_vel", max_vel_x_, max_vel_x_);
  nh_->param("/max_ang_vel", max_rotational_vel_, max_rotational_vel_);
  nh_->param<std::string>("/cmd_topic", cmd_topic_, cmd_topic_);

  vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/gazebo/cmd_vel", 1);
  joy_sub_ = nh_->subscribe<sensor_msgs::Joy>("/joy", 10, &RobotJoyTeleop::joyCallback, this);
  verification_pub_ = nh_->advertise<std_msgs::Int32>("/teleop_verification", 1);
}

void RobotJoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  speed_multiplier_ = 1.0;
  // check if cross is used for steering
  vel.linear.x = max_vel_x_ * joy->axes[1] * speed_multiplier_;
  vel.angular.z = max_rotational_vel_ * joy->axes[0] * speed_multiplier_;
  vel_pub_.publish(vel);

  std_msgs::Int32 msg;
  msg.data = 1;
  verification_pub_.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  RobotJoyTeleop robot_joy_teleop;
  ros::spin();
}

