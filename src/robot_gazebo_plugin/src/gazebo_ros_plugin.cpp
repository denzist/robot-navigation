#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <common/Plugin.hh>
#include <common/Time.hh>
#include <common/Events.hh>
#include <physics/physics.hh>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#define NUM_JOINTS 4


namespace gazebo
{   
  class GazeboRosPlugin : public ModelPlugin
  {

    public: GazeboRosPlugin()
    {

      // Start up ROS
      std::string name = "gazebo_ros_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);

    }
    public: ~GazeboRosPlugin()
    {
      this->node->shutdown();
      delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      if (!this->model)
      {
        ROS_FATAL("GazeboRosControl need Model!");
        return;
      }
      this->world = _parent->GetWorld();
      if (!this->world)
      {
        ROS_FATAL("GazeboRosControl can't get world!");
        return;
      }
      this->robot_namespace = "";
      if (_sdf->HasElement("robotNamespace"))
      {
        this->robot_namespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
      }

      this->cmd_vel_topic_name = "/cmd_vel";
      if (_sdf->HasElement("cmd_vel_topic_name"))
        this->cmd_vel_topic_name = _sdf->GetElement("cmd_vel_topic_name")->GetValueString();

      this->odom_topic_name = "/odom";
      if (_sdf->HasElement("odom_topic_name"))
        this->odom_topic_name = _sdf->GetElement("odom_topic_name")->GetValueString();

      this->joint_states_topic_name = "/joint_states";
      if (_sdf->HasElement("joint_states_topic_name"))
        this->joint_states_topic_name = _sdf->GetElement("joint_states_topic_name")->GetValueString();

      //init 
      this->joint_states.name.resize(NUM_JOINTS);
      this->joint_states.position.resize(NUM_JOINTS);
      this->joint_states.velocity.resize(NUM_JOINTS);
      this->joint_states.effort.resize(NUM_JOINTS);

      for (int i = 0; i < NUM_JOINTS; ++i)
      {
        this->joint_states.position[i] = 0;
        this->joint_states.velocity[i] = 0;
        this->joint_states.effort[i] = 0;
      }

      this->joint_states.name[0] = "left_front_wheel_joint";
      if (_sdf->HasElement("left_front_wheel_joint"))
        this->joint_states.name[0] = _sdf->GetElement("left_front_wheel_joint")->GetValueString();

      this->joint_states.name[1] = "left_rear_wheel_joint";
      if (_sdf->HasElement("left_rear_wheel_joint"))
        this->joint_states.name[1] = _sdf->GetElement("left_rear_wheel_joint")->GetValueString();

      this->joint_states.name[2] = "right_front_wheel_joint";
      if (_sdf->HasElement("right_front_wheel_joint"))
        this->joint_states.name[2] = _sdf->GetElement("right_front_wheel_joint")->GetValueString();

      this->joint_states.name[3] = "right_rear_wheel_joint";
      if (_sdf->HasElement("right_rear_wheel_joint"))
        this->joint_states.name[3] = _sdf->GetElement("right_rear_wheel_joint")->GetValueString();

      this->wheel_sep = 0.5;
      if (_sdf->HasElement("wheel_separation"))
        this->wheel_sep = _sdf->GetElement("wheel_separation")->GetValueDouble();

      this->wheel_radius = 0.1;
      if (_sdf->HasElement("wheel_radius"))
        this->wheel_radius = _sdf->GetElement("wheel_radius")->GetValueDouble();

      this->torque = 4.0;
      if (_sdf->HasElement("torque"))
        this->torque = _sdf->GetElement("torque")->GetValueDouble();

      this->max_velocity = 4.0;
      if (_sdf->HasElement("max_velocity"))
        this->max_velocity = _sdf->GetElement("max_velocity")->GetValueDouble();


      // ROS Nodehandle
      this->node = new ros::NodeHandle("~");

      // ROS Subscriber and Publishers
      this->cmd_vel_sub = this->node->subscribe("cmd_vel", 1,
        &GazeboRosPlugin::ROSCmdVelCallback, this);
      this->odom_pub = this->node->advertise<nav_msgs::Odometry>(this->odom_topic_name, 1);
      this->joint_state_pub = this->node->advertise<sensor_msgs::JointState>(this->joint_states_topic_name, 1);


      for (int i = 0; i < NUM_JOINTS; ++i)
      {
        this->joints[i] = this->model->GetJoint(this->joint_states.name[i]);
        if (!this->joints[i])
          gzthrow("The controller couldn't get joint " << this->joint_states.name[i]);
      }

      //initialize time and odometry position
      this->prev_update_time = this->last_cmd_vel_time = this->world->GetSimTime();
      this->robot_pose_x = 0.0;
      this->robot_pose_y = 0.0;
      this->robot_pose_th = 0.0;
      this->robot_vx = 0.0;
      this->robot_vy = 0.0;
      this->robot_vth = 0.0;

      // Get then name of the model
      std::string modelName = _sdf->GetParent()->GetValueString("name");
      
      ROS_INFO("gazebo_ros_control plugin initialized");


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ros::spinOnce();
      for (int i = 0; i < NUM_JOINTS; i++)
        this->joints[i]->SetMaxForce(0, this->torque);


      common::Time current_time = this->world->GetSimTime();
      common::Time delta_t = current_time - this->prev_update_time;
      this->prev_update_time = current_time;

      double v_left = ((this->joints[0]->GetVelocity(0) + this->joints[1]->GetVelocity(0)) / 2.0) * (this->wheel_radius); //! need to test !
      double v_right = ((this->joints[2]->GetVelocity(0) + this->joints[3]->GetVelocity(0)) / 2.0) * (this->wheel_radius); //! need to test!
      this->robot_linear_speed = (v_right + v_left) / 2.0; 
      this->robot_angular_speed = (v_right - v_left) / this->wheel_sep; 

      // turn left wheels
      this->joints[0]->SetVelocity(0, this->wheel_speed_left / this->wheel_radius);
      this->joints[1]->SetVelocity(0, this->wheel_speed_left / this->wheel_radius);

      // turn right wheels
      this->joints[2]->SetVelocity(0, this->wheel_speed_right / this->wheel_radius);
      this->joints[3]->SetVelocity(0, this->wheel_speed_right / this->wheel_radius);

      //estimate odom
      this->robot_vx = this->robot_linear_speed * cos(this->robot_pose_th);
      this->robot_vy = this->robot_linear_speed * sin(this->robot_pose_th);
      this->robot_vth = this->robot_angular_speed;

      double dt = delta_t.Double();
      this->robot_pose_x += this->robot_vx * dt;
      this->robot_pose_y += this->robot_vy * dt;
      this->robot_pose_th += this->robot_vth * dt;

      publishOdom(current_time);
      publishJointStates(current_time);

    }

    void ROSCmdVelCallback(const geometry_msgs::Twist &msg)
    {

      gzmsg << "cmd_vel: " << msg.linear.x << ", "
         << msg.angular.z << std::endl;
      this->last_cmd_vel_time = this->world->GetSimTime();
      double vl = msg.linear.x;
      double va= msg.angular.z;

      this->wheel_speed_left = (vl - va * this->wheel_sep/2)/this->wheel_radius;
      this->wheel_speed_right = (vl + va * this->wheel_sep/2)/this->wheel_radius;

      if (fabs(this->wheel_speed_left) > this->max_velocity)
        this->wheel_speed_left = copysign(this->max_velocity, this->wheel_speed_left);
      if (fabs(this->wheel_speed_right) > this->max_velocity)
        this->wheel_speed_right = copysign(this->max_velocity, this->wheel_speed_right);
    }

    void publishOdom(common::Time &current_time)
    {
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(this->robot_pose_th);

      nav_msgs::Odometry odom;
      odom.header.stamp.sec = current_time.sec;
      odom.header.stamp.nsec = current_time.nsec;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = this->robot_pose_x;
      odom.pose.pose.position.y = this->robot_pose_y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = this->robot_vx;
      odom.twist.twist.linear.y = this->robot_vy;
      odom.twist.twist.angular.z = this->robot_vth;

      //publish odom
      this->odom_pub.publish(odom);
    }

    void publishJointStates(common::Time &current_time)
    {
      this->joint_states.header.stamp.sec = current_time.sec;
      this->joint_states.header.stamp.nsec = current_time.nsec;

      for (size_t i = 0; i < NUM_JOINTS; ++i)
      {
        this->joint_states.position[i] = this->joints[i]->GetAngle(0).Radian();
        this->joint_states.velocity[i] = this->joints[i]->GetVelocity(0);
      }

      this->joint_state_pub.publish(this->joint_states);
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Subscribers and Publishers
    private: ros::Publisher odom_pub;
    private: ros::Publisher joint_state_pub;
    private: ros::Subscriber cmd_vel_sub;

    private: std::string robot_namespace;

    private: std::string cmd_vel_topic_name;
    private: std::string odom_topic_name;
    private: std::string joint_states_topic_name;

    private: double wheel_sep;
    private: double wheel_radius;

    /// maximum torque applied to the wheels [Nm]
    private: double torque;

    /// maximum linear speed of the robot [m/s]
    private: double max_velocity;

    /// Pointers to the model and world  
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// Desired speeds of the wheels
    private: double wheel_speed_right;
    private: double wheel_speed_left;

    private: physics::JointPtr joints[NUM_JOINTS];

    // Simulation time of the last update
    private: common::Time prev_update_time;

    // Simulation time when the last cmd_vel command was received (for timeout)
    private: common::Time last_cmd_vel_time;

    /// robot pose
    private: double robot_pose_x;
    private: double robot_pose_y;
    private: double robot_pose_th;

    private: double robot_vx;
    private: double robot_vy;
    private: double robot_vth;

    ///robot speed
    private: double robot_linear_speed;
    private: double robot_angular_speed;


    private: sensor_msgs::JointState joint_states;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosPlugin)
}