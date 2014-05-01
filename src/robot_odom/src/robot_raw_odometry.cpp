#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

//robots 6DOF pose
  double robot_pose_x = 0.0;
  double robot_pose_y = 0.0;
  double robot_pose_th = 0.0;

  double robot_vx = 0.0;
  double robot_vy = 0.0;
  double robot_vth = 0.0;

  //local linear and angular speeds
  double robot_linear_speed = 0.0; //meeters per second
  double robot_angular_speed = 0.0; // rad per second

  //robot's characteristic numbers
  double ROBOT_WHEEL_RADIUS = 0.1;
  double ROBOT_WHEEL_SEP = 0.5;

  //velocity vector which will be used for saving inf about joint states
  std::vector<double> wheel_speed;



void jointStatesCallback(const sensor_msgs::JointState& joint_states) {

  //vecPos = joint_states.position;
  wheel_speed = joint_states.velocity;
  //vecEff = joint_states.effort;

}

int main(int argc, char** argv){

  ros::init(argc, argv, "row_odometry_publisher");
  ROS_INFO("raw_odometry_publisher node loaded!");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/raw_odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Subscriber joint_subscriber = n.subscribe("/gazebo/joint_states", 1, jointStatesCallback);

  
  wheel_speed.assign(4, 0.0);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0);
  while(n.ok()){

    ros::spinOnce();      // check for incoming messages
    current_time = ros::Time::now();

    //calculating odom


    double v_left = ((wheel_speed[0] + wheel_speed[2]) / 2.0) * (ROBOT_WHEEL_RADIUS); //! need to test !
    double v_right = ((wheel_speed[1] + wheel_speed[3]) / 2.0) * (ROBOT_WHEEL_RADIUS); //! need to test!
    robot_linear_speed = (v_right + v_left) / 2.0; 
    robot_angular_speed = (v_right - v_left) / ROBOT_WHEEL_SEP; 

    robot_vx = robot_linear_speed * cos(robot_pose_th);
    robot_vy = robot_linear_speed * sin(robot_pose_th);
    robot_vth = robot_angular_speed;

    double dt = (current_time - last_time).toSec();
    robot_pose_x += robot_vx * dt;
    robot_pose_y += robot_vy * dt;
    robot_pose_th += robot_vth * dt;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pose_th);
    //publish TF

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "raw_odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = robot_pose_x;
    odom_trans.transform.translation.y = robot_pose_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "raw_odom";

    //set the position
    odom.pose.pose.position.x = robot_pose_x;
    odom.pose.pose.position.y = robot_pose_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = robot_vx;
    odom.twist.twist.linear.y = robot_vy;
    odom.twist.twist.angular.z = robot_vth;

    for(int i = 0; i < 6; i++)
        odom.pose.covariance[i*6+i] = 0.1;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}