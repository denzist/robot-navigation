#ifndef GOAL_CONTROLLER_H_
#define GOAL_CONTROLLER_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalController
{
public:
	GoalController();
	~GoalController();
	//bool isEqual(const robot_msgs::Goal &goal);
	void sendGoal(const move_base_msgs::MoveBaseGoal& goal);
	MoveBaseClient* move_base_client_;
	inline bool isStarted();
	inline void setStarted();
private:
	bool is_started_;
};

inline bool GoalController::isStarted()
{
	return is_started_;
}

inline void GoalController::setStarted()
{
	is_started_ = true;
}

GoalController::GoalController()
{
	move_base_client_ = new MoveBaseClient("move_base", true);
	while(!move_base_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	is_started_ = false;
}

GoalController::~GoalController()
{
	delete move_base_client_;
}

void GoalController::sendGoal(const move_base_msgs::MoveBaseGoal& goal)
{
	move_base_client_->sendGoal(goal);
	return;
}

#endif