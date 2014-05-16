#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <vector>
#include <robot_msgs/Goal.h>
#include <std_msgs/Int32.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher goal_status_pub;
ros::Subscriber goal_sub;
robot_msgs::Goal new_goal;
robot_msgs::Goal current_goal;



#define _INACTIVE 0
#define _ACTIVE 1
#define _SUCCEEDED 2
#define _ABORTED 3

bool isEqual(robot_msgs::Goal &new_goal, robot_msgs::Goal current_goal){
  if(current_goal.goal.x != new_goal.goal.x){
    return false;
  }
  if(current_goal.goal.y != new_goal.goal.y){
    return false;
  }
  if(current_goal.goal.z != new_goal.goal.z){
    return false;
  }
  return true;

}

void sendGoal(MoveBaseClient &ac){
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = new_goal.goal.x;
  goal.target_pose.pose.position.y = new_goal.goal.y;
  goal.target_pose.pose.position.z = new_goal.goal.z;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal x=%f y=%f z=%f ", new_goal.goal.x, 
            new_goal.goal.y, new_goal.goal.z);

  ac.sendGoal(goal);

  current_goal.status = new_goal.status;
  current_goal.goal.x = new_goal.goal.x;
  current_goal.goal.y = new_goal.goal.y;
  current_goal.goal.z = new_goal.goal.z;
}



void EmergencyMove(MoveBaseClient &ac){
  std_msgs::Int32 controller_status;

  //controller recieve the active goal
  if(new_goal.status == _ACTIVE){
    //if current goal still active
    if(current_goal.status == _ACTIVE){
      if(!isEqual(new_goal, current_goal)){
        sendGoal(ac);
      }else{
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("Succeded goal x=%f y=%f z=%f ",current_goal.goal.x, 
            current_goal.goal.y, current_goal.goal.z);
          current_goal.status = _SUCCEEDED;

        }
        if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
          ROS_INFO("The base failed goal x=%f y=%f z=%f ",current_goal.goal.x, 
            current_goal.goal.y, current_goal.goal.z);
          current_goal.status = _ABORTED;
        }
      }
    }else{
      //if current goal is inactive
      if(current_goal.status == _INACTIVE){
        sendGoal(ac);
      }
    }
  }else{
    //if teleop tracker sended inactive state
    if(current_goal.status == _ACTIVE){
      ROS_INFO("Aborting current goal");
      ac.cancelGoal();
      current_goal.status = _INACTIVE;
    }
    if(current_goal.status == _SUCCEEDED || current_goal.status == _ABORTED){
      ac.cancelGoal();
      current_goal.status = _INACTIVE;
    }
  }
  //send the teleop tracker goal status
  controller_status.data = current_goal.status;
  goal_status_pub.publish(controller_status);
  return;
}

void GoalCallback(const robot_msgs::Goal &msg){
  new_goal = msg;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gaol_controller");
  ros::NodeHandle n;

  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  current_goal.status = _INACTIVE;

  ros::Rate rate(100.0);
  goal_status_pub = n.advertise<std_msgs::Int32>("/emergency_controller/goal_status", 1);
  goal_sub = n.subscribe("/emergency_controller/goal", 10, GoalCallback);

  while(n.ok()){
    ros::spinOnce();
    EmergencyMove(ac);
    rate.sleep();
  }

  return 0;
}