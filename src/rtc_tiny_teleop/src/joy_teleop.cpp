#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>


class RTCTinyJoyTeleop
{
public:
	RTCTinyJoyTeleop();

private:
	void callback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle* nh_;
	//absolute values > 0.0
	double max_lin_vel_;
	double max_ang_vel_;
	double lin_step_;
	double ang_step_;
	double cur_lin_vel_; 
	double cur_ang_vel_;
	bool cruise_control_;
	ros::Time last_update_;
	std::string cmd_topic_;
	geometry_msgs::Twist vel_;

	ros::Publisher vel_pub_;
	ros::Publisher verification_pub_;
	ros::Subscriber joy_sub_;
};


RTCTinyJoyTeleop::RTCTinyJoyTeleop():
cur_lin_vel_(3.0),
cur_ang_vel_(0.2),
lin_step_(3.0),
ang_step_(0.2),
max_lin_vel_(30.0),
max_ang_vel_(2.0),
cmd_topic_("/cmd_vel"),
cruise_control_(false)
{
	nh_ = new ros::NodeHandle("~");
	last_update_ = ros::Time::now();
	nh_->param("/vel_step", lin_step_, lin_step_);
	nh_->param("/ang_step", ang_step_, ang_step_);
	nh_->param("/max_lin_vel", max_lin_vel_, max_lin_vel_);
	nh_->param("/max_ang_vel", max_ang_vel_, max_ang_vel_);
	nh_->param<std::string>("/cmd_topic", cmd_topic_, cmd_topic_);
	vel_pub_ = nh_->advertise<geometry_msgs::Twist>(cmd_topic_, 1);
	joy_sub_ = nh_->subscribe<sensor_msgs::Joy>("/joy", 10, &RTCTinyJoyTeleop::callback, this);
	verification_pub_ = nh_->advertise<std_msgs::Int32>("/teleop_verification", 1);
}

void RTCTinyJoyTeleop::callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	ros::Duration delta = ros::Time::now() - last_update_;
	if(delta.toSec() > 0.1)
	{
		last_update_ = ros::Time::now();
		if(joy->buttons[4])
		{
			cruise_control_ = !cruise_control_;
			ROS_INFO_STREAM("cruise_control_ = " << cruise_control_);
		}
		if(joy->buttons[0] && cur_lin_vel_ - lin_step_ >= 0.0) //A
		{
			cur_lin_vel_ -= lin_step_;
			ROS_INFO_STREAM("cur_lin_vel_ = "<<cur_lin_vel_);
		}
		if(joy->buttons[3] && cur_lin_vel_ + lin_step_ <= max_lin_vel_) //Y
		{
			cur_lin_vel_ += lin_step_;
			ROS_INFO_STREAM("cur_lin_vel_ = "<<cur_lin_vel_);
		}
		if(joy->buttons[2] && cur_ang_vel_ - ang_step_ >= 0.0) //X
		{
			cur_ang_vel_ -= ang_step_;
			ROS_INFO_STREAM("cur_ang_vel_ = "<<cur_ang_vel_);
		}
		if(joy->buttons[1] && cur_ang_vel_ + ang_step_ <= max_ang_vel_) //B
		{
			cur_ang_vel_ +=ang_step_;
			ROS_INFO_STREAM("cur_ang_vel_ = "<<cur_ang_vel_);
		}
	}
	if(cruise_control_)
	{
		vel_pub_.publish(vel_);
		std_msgs::Int32 msg;
		msg.data = 1;
		verification_pub_.publish(msg);
		return;
	}
	vel_.linear.x = cur_lin_vel_ * joy->axes[7];
	vel_.angular.z = cur_ang_vel_ * joy->axes[6];
	vel_pub_.publish(vel_);
	std_msgs::Int32 msg;
	msg.data = 1;
	verification_pub_.publish(msg);
	return;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_teleop");
	RTCTinyJoyTeleop j_teleop;
	ros::spin();
	return 0;
}

