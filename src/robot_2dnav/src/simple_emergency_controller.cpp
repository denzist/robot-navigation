#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <vector>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::vector<tf::StampedTransform> transform_v;
boost::shared_ptr<geometry_msgs::TwistStamped const> sharedPtr;
bool emergency_mode;


double Distance(tf::StampedTransform& tf1, tf::StampedTransform& tf2){
  return sqrt((tf1.getOrigin().x() - tf2.getOrigin().x())*(tf1.getOrigin().x() - tf2.getOrigin().x())+
              (tf1.getOrigin().y() - tf2.getOrigin().y())*(tf1.getOrigin().y() - tf2.getOrigin().y())+
              (tf1.getOrigin().z() - tf2.getOrigin().z())*(tf1.getOrigin().z() - tf2.getOrigin().z())
              );
}


void CmdVelCallback(){
  //if cmd_vel is updating
  sharedPtr  = ros::topic::waitForMessage<geometry_msgs::TwistStamped>("/time_stamp_cmd_vel", ros::Duration(4.0));
  if(sharedPtr != NULL){
    tf::StampedTransform current_transform;
    tf::TransformListener listener;
    try{
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/map", "/base_link",
                                 ros::Time(0), current_transform);
        /*ROS_INFO(" x=%f y=%f z=%f child=%s frameId=%s ", current_transform.getOrigin().x(), 
        current_transform.getOrigin().y(), current_transform.getOrigin().z(), current_transform.frame_id_.c_str(), 
        current_transform.child_frame_id_.c_str());//,  transform.getOrigin().yaw()); */
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return;
    }
    if(transform_v.empty()){
      transform_v.push_back(current_transform);
      return;
    }
    if(transform_v.size() == 1){
      transform_v.push_back(current_transform);
      return;
    }
    int last_i = transform_v.size() - 1;
    double d1 = Distance(current_transform, transform_v[last_i]);
    double d2 = Distance(current_transform, transform_v[last_i - 1]);
    if(d1 >= 6.0 && d2 >= 12.0){
      transform_v.push_back(current_transform);
      return;
    }
    if(d1 >= 6.0 && d2 <= 12.0){
      transform_v[last_i] = current_transform;
      return;
    }
  }else{
  //if teleoparating turned off
    emergency_mode = true;
    return;
  }
}

void EmergencyMove(){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  for(int i = transform_v.size() - 1; i >= 0 ; --i){
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = transform_v[i].getOrigin().x();
    goal.target_pose.pose.position.y = transform_v[i].getOrigin().y();
    goal.target_pose.pose.position.z = transform_v[i].getOrigin().z();
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal x=%f y=%f z=%f child=%s frameId=%s ", transform_v[i].getOrigin().x(), 
        transform_v[i].getOrigin().y(), transform_v[i].getOrigin().z(), transform_v[i].frame_id_.c_str(), 
        transform_v[i].child_frame_id_.c_str());

    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Succeded goal x=%f y=%f z=%f child=%s frameId=%s ", transform_v[i].getOrigin().x(), 
        transform_v[i].getOrigin().y(), transform_v[i].getOrigin().z(), transform_v[i].frame_id_.c_str(), 
        transform_v[i].child_frame_id_.c_str());
    else
      ROS_INFO("The base failed goal x=%f y=%f z=%f child=%s frameId=%s ", transform_v[i].getOrigin().x(), 
        transform_v[i].getOrigin().y(), transform_v[i].getOrigin().z(), transform_v[i].frame_id_.c_str(), 
        transform_v[i].child_frame_id_.c_str());
  }
  return;
}

//void emergency_move();

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  emergency_mode = false;
  ros::Rate rate(100.0);
  while(n.ok()){
    ros::spinOnce();
    CmdVelCallback();
    rate.sleep();
  }
  if(emergency_mode){
    int size = transform_v.size();
    ROS_INFO("Size of transform_v = %d\n", size);
    for(int i = 0; i < transform_v.size(); ++i){
      ROS_INFO(" x=%f y=%f z=%f child=%s frameId=%s ", transform_v[i].getOrigin().x(), 
        transform_v[i].getOrigin().y(), transform_v[i].getOrigin().z(), transform_v[i].frame_id_.c_str(), 
        transform_v[i].child_frame_id_.c_str());
    }

    EmergencyMove();
    
  }
  return 0;
}