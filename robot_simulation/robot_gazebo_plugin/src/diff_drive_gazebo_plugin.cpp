#include "diff_drive_gazebo_plugin.h"
#include <boost/lexical_cast.hpp>

const int PI = 3.1415926535897931;

namespace gazebo
{   
    DiffDrivePlugin::DiffDrivePlugin()
    {
      std::string name = "diff_drive_gazebo_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);
    }

    DiffDrivePlugin::~DiffDrivePlugin()
    {
      node_->shutdown();
      delete node_;
    }

    void DiffDrivePlugin::parseSDF(sdf::ElementPtr sdf){
    	cmd_vel_topic_ = sdf->Get<std::string>("cmd_vel_topic_name");
    	if(cmd_vel_topic_ == "")
    		cmd_vel_topic_ = "cmd_vel";
    	odom_topic_ = sdf->Get<std::string>("odom_topic_name");
    	if(odom_topic_ == "")
    	   	odom_topic_ = "odom";
    	joint_states_topic_ = sdf->Get<std::string>("joint_states_topic_name"); 
    	if(joint_states_topic_ == "")
    		joint_states_topic_= "joint_states";
    	//parse joints
    	r_num_joints_ = sdf->Get<int>("num_joints");
    	if(r_num_joints_ == 0)
    		r_num_joints_ = 4;
    	r_joint_states_.name.resize(r_num_joints_);
    	r_joint_states_.position.resize(r_num_joints_);
    	r_joint_states_.velocity.resize(r_num_joints_);
    	r_joint_states_.effort.resize(r_num_joints_);
    	for (int i = 0; i < r_num_joints_; ++i)
    	{
    		r_joint_states_.position[i] = 0;
    		r_joint_states_.velocity[i] = 0;
    		r_joint_states_.effort[i] = 0;
    	}
    	//left side
    	for(int i = 0; i < r_num_joints_/2; ++i){
    		std::string str_i = boost::lexical_cast<std::string>(i);
    		r_joint_states_.name[i] = sdf->Get<std::string>("left_wheel_" + str_i);
    	}
    	//right side
    	for(int i = r_num_joints_/2; i < r_num_joints_; ++i){
    		std::string str_i = boost::lexical_cast<std::string>(i);
    		r_joint_states_.name[i] = sdf->Get<std::string>("right_wheel_" + str_i);
    	}
    	r_w_sep_ = sdf->Get<double>("wheel_separation");
    	if(r_w_sep_ == *(new double()))
    		r_w_sep_ = 0.5;
    	r_w_rad_ = sdf->Get<double>("wheel_radius");
    	if(r_w_rad_ == *(new double()))
    		r_w_rad_ = 0.1;
    	r_w_torq_ = sdf->Get<double>("torque");
    	if(r_w_torq_ == *(new double()))
    		r_w_torq_ = 10.0;
    	r_max_lin_vel_ = sdf->Get<double>("max_velocity");
    	if(r_max_lin_vel_ == *(new double()))
    		r_max_lin_vel_ = 4.0;
    }

    void DiffDrivePlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
	    model_ = parent;
    	if (!model_)
    	{
    		ROS_FATAL("GazeboRosControl need Model!");
    		return;
    	}
    	world_ = parent->GetWorld();
    	if (!world_)
    	{
    		ROS_FATAL("GazeboRosControl can't get world!");
    		return;
    	}
    	parseSDF(sdf);
    	r_joints_ = new physics::JointPtr[r_num_joints_];
    	for (int i = 0; i < r_num_joints_; ++i)
    	{
    		r_joints_[i] = model_->GetJoint(r_joint_states_.name[i]);
    		if (r_joints_[i] == NULL)
    			gzthrow("The controller couldn't get joint " << r_joint_states_.name[i]);
    	}
    	/*prv_update_t_  = prv_print_t_  = world_->GetSimTime();
    	r_x_ = 0.0;
    	r_y_ = 0.0;
    	r_th_ = 0.0;
    	r_vx_ = 0.0;
    	r_vy_ = 0.0;
    	r_vth_ = 0.0;
		  */
    	node_ = new ros::NodeHandle("~");
    	cmd_vel_sub_ = node_->subscribe(cmd_vel_topic_, 1,
    		&DiffDrivePlugin::callbackCmdVel, this);
    	//ground_truth_pose_pub_ = node_->advertise<nav_msgs::Odometry>("base_pose_ground_truth", 1);
    	joint_state_pub_ = node_->advertise<sensor_msgs::JointState>(joint_states_topic_, 1);

    	base_link_ = model_->GetLink("base_link");
    	if (base_link_ == NULL)
    		gzthrow("The controller couldn't get base_link " << base_link_);
    	ROS_INFO("gazebo_ros_control plugin initialized");
	    // Listen to the update event. This event is broadcast every
	    // simulation iteration.
    	update_connection_ = event::Events::ConnectWorldUpdateBegin(
    		boost::bind(&DiffDrivePlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void DiffDrivePlugin::OnUpdate()
    {
      ros::spinOnce();
      publishJointStates();
    //  publishGroundTruth(current_t);
    }

    void DiffDrivePlugin::callbackCmdVel(const geometry_msgs::Twist &cmd_vel)
    {
    	
    	double vel_lin = cmd_vel.linear.x / r_w_rad_;
      double k = 1.26402;
      double vel_ang = cmd_vel.angular.z*k*r_w_sep_ / 2 / r_w_rad_;
      for (int i = 0; i < r_num_joints_; i++)
       	r_joints_[i]->SetMaxForce(0, r_w_torq_);
      for (int i = 0; i < r_num_joints_/2; i++)
      	r_joints_[i]->SetVelocity(0, vel_lin - vel_ang);
     	for (int i = r_num_joints_/2; i < r_num_joints_; i++)
     		r_joints_[i]->SetVelocity(0, vel_lin + vel_ang);
		
    }
/*
    void publishGroundTruth(common::Time &current_time)
    {

      nav_msgs::Odometry odom;
      odom.header.stamp.sec = current_time.sec;
      odom.header.stamp.nsec = current_time.nsec;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      math::Pose base_link_pose = this->base_link->GetWorldPose();
      odom.pose.pose.position.x = base_link_pose.pos.x;
      odom.pose.pose.position.y = base_link_pose.pos.y;
      odom.pose.pose.position.z = base_link_pose.pos.z;
      odom.pose.pose.orientation.x = base_link_pose.rot.x;
      odom.pose.pose.orientation.y = base_link_pose.rot.y;
      odom.pose.pose.orientation.z = base_link_pose.rot.z;
      odom.pose.pose.orientation.w = base_link_pose.rot.w;

      math::Vector3 linear_vel = this->base_link->GetWorldLinearVel(); 
      math::Vector3 angular_vel = this->base_link->GetWorldAngularVel(); 
      odom.twist.twist.linear.x = linear_vel.x;
      odom.twist.twist.linear.y = linear_vel.y;
      odom.twist.twist.linear.z = linear_vel.z;
      odom.twist.twist.angular.x = angular_vel.x;
      odom.twist.twist.angular.y = angular_vel.y;
      odom.twist.twist.angular.z = angular_vel.z;
      this->robot_linear_speed = sqrt(linear_vel.x * linear_vel.x + linear_vel.y*linear_vel.y);
      this->robot_angular_speed = angular_vel.z;
      this->ground_truth_pose_pub.publish(odom);
    }
*/
    void DiffDrivePlugin::publishJointStates()
    {
      common::Time current_t = world_->GetSimTime();
      r_joint_states_.header.stamp.sec = current_t.sec;
      r_joint_states_.header.stamp.nsec = current_t.nsec;
     	for (int i = 0; i < r_num_joints_; ++i){
        r_joint_states_.position[i] = r_joints_[i]->GetAngle(0).Radian();
        r_joint_states_.velocity[i] = r_joints_[i]->GetVelocity(0);
      }
      joint_state_pub_.publish(r_joint_states_);
    }
}