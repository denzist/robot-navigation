#ifndef DIFF_DRIVE_GAZEBO_PLUGIN_H_
#define DIFF_DRIVE_GAZEBO_PLUGIN_H_

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{   
  class DiffDrivePlugin : public ModelPlugin
  {
  public:
    DiffDrivePlugin();
    ~DiffDrivePlugin();
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
    void OnUpdate();
    void callbackCmdVel(const geometry_msgs::Twist &cmd_vel);
  //  void publishGroundTruth(common::Time &current_time);
    void publishJointStates();
    void parseSDF(sdf::ElementPtr sdf);
  private:
    event::ConnectionPtr update_connection_; // Pointer to the update event connection
    ros::NodeHandle* node_;  // ROS Nodehandle
    ros::Publisher joint_state_pub_; // ROS Subscribers and Publishers
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher ground_truth_pose_pub_;
    //r_ - robot 
    std::string cmd_vel_topic_; // topic names
    std::string odom_topic_; 
    std::string joint_states_topic_;

    double r_w_sep_; // wheel separation [m]
    double r_w_rad_; // wheel radius [m]
    double r_w_torq_; // maximum torque applied to the wheels [Nm]
    double r_max_lin_vel_; // maximum linear speed of the robot [m/s]
    /*double r_x_; // robot pose
    double r_y_;
    double r_th_;
    double r_vx_;
    double r_vy_;
    double r_vth_;
    double r_lin_vel_;  //robot lin and ang speed
    double r_ang_vel_;*/
    int r_num_joints_;
    
    physics::WorldPtr world_; // pointers to the model and world  
    physics::ModelPtr model_;
    physics::LinkPtr base_link_;
    physics::JointPtr *r_joints_;
    sensor_msgs::JointState r_joint_states_;

    //common::Time prv_update_t_; // simulation time of the last update
    //common::Time prv_print_t_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)
}

#endif