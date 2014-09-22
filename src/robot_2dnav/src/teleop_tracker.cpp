#include "autonomous_return.h"


#define _INACTIVE 0
#define _ACTIVE 1
#define _SUCCEEDED 2
#define _ABORTED 3


std::vector<tf::StampedTransform> transform_v;
boost::shared_ptr<std_msgs::Int32 const> sharedPtr;
ros::Publisher goal_pub;
ros::Subscriber goal_status_sub;
int goal_status;

double Distance(tf::StampedTransform& tf1, tf::StampedTransform& tf2){
  return sqrt((tf1.getOrigin().x() - tf2.getOrigin().x())*(tf1.getOrigin().x() - tf2.getOrigin().x())+
              (tf1.getOrigin().y() - tf2.getOrigin().y())*(tf1.getOrigin().y() - tf2.getOrigin().y())+
              (tf1.getOrigin().z() - tf2.getOrigin().z())*(tf1.getOrigin().z() - tf2.getOrigin().z())
              );
}


void CmdVelCallback(){
  //if cmd_vel is updating
  sharedPtr  = ros::topic::waitForMessage<std_msgs::Int32>("/teleop_veification", ros::Duration(4.0));
  if(sharedPtr != NULL){

    robot_msgs::Goal goal;
    //turn goal controller in inactive state 
    goal.status = _INACTIVE;
    goal_pub.publish(goal);
    //starting  listening tf for saving some longway points
    tf::StampedTransform current_transform;
    tf::TransformListener listener;
    try{
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/map", "/base_link",
                                 ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return;
    }
    //collecting longway points
    if(transform_v.empty()){
      transform_v.push_back(current_transform);
      ROS_INFO("The queue was empty. Added goal x=%f y=%f z=%f ",current_transform.getOrigin().x(), 
            current_transform.getOrigin().y(), current_transform.getOrigin().z());
      return;
    }
    int last_i = transform_v.size() - 1;
    double d1 = Distance(current_transform, transform_v[last_i]);
    if(transform_v.size() == 1 && d1 >= 10.0){
      transform_v.push_back(current_transform);
      ROS_INFO("The queue had 1 element. Added goal x=%f y=%f z=%f ",current_transform.getOrigin().x(), 
            current_transform.getOrigin().y(), current_transform.getOrigin().z());
      return;
    }
    if(transform_v.size() >= 2){
    	double d2 = Distance(current_transform, transform_v[last_i - 1]);
    	if(d1 >= 10.0 && d2 >= 10.0){
      		transform_v.push_back(current_transform);
      		ROS_INFO("Added new goal x=%f y=%f z=%f ",current_transform.getOrigin().x(), 
            	current_transform.getOrigin().y(), current_transform.getOrigin().z());
      		return;
    	}
    	if(d1 >= 10.0 && d2 <= 10.0){
      		transform_v[last_i] = current_transform;
      		ROS_INFO("Current position is closer than queue last position. Replaced with goal x=%f y=%f z=%f ",current_transform.getOrigin().x(), 
            	current_transform.getOrigin().y(), current_transform.getOrigin().z());
      		return;
    	}
    	if(d1 <= 10.0 && d2 <=10.0){
    		transform_v.pop_back();
    		ROS_INFO("Current position is closer than queue last position. Deleting last goal x=%f y=%f z=%f ",transform_v.back().getOrigin().x(), 
            transform_v.back().getOrigin().y(), transform_v.back().getOrigin().z());
    		return;
    	}
    }
    return;
  }else{
  //if teleoparating turned off
    //send active state to goal controller
    if(transform_v.size() != 0){
    if(goal_status == _INACTIVE){
      robot_msgs::Goal goal;
      goal.status = _ACTIVE;
      goal.goal.x = transform_v.back().getOrigin().x();
      goal.goal.y = transform_v.back().getOrigin().y();
      goal.goal.z = transform_v.back().getOrigin().z();
      goal_pub.publish(goal);
      return;
    }
    //listen goal controller if it achieved the goal, after turn it in inactive state 
    if(goal_status == _SUCCEEDED || goal_status == _ABORTED){
      if(!transform_v.empty()){
        transform_v.pop_back();
      }else{
        ROS_INFO("The goal queue is empty");
      }
      robot_msgs::Goal goal;
      goal.status = _INACTIVE;
      goal_pub.publish(goal);
      return;
    }
	}
    //if current goal is still active
    return;
  }
}

void GoalStatusCallback(const std_msgs::Int32 &msg){
  goal_status = msg.data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "teleop_tracker");
  ros::NodeHandle n;
  ros::Rate rate(100.0);
  goal_pub = n.advertise<robot_msgs::Goal>("/emergency_controller/goal", 1);
  goal_status_sub = n.subscribe("/emergency_controller/goal_status", 10, GoalStatusCallback);
  while(n.ok()){
    ros::spinOnce();
    CmdVelCallback();
    rate.sleep();
  }
  return 0;
}