#ifndef AUTONOMOUS_RETURN_H_
#define AUTONOMOUS_RETURN_H_

#include "goal_controller.h"
#include "pose_vector.h"
#include <std_msgs/Int32.h>


class AutonomousReturnSystem : private PoseVector<tf::StampedTransform>, private GoalController
{
public:
	AutonomousReturnSystem();
	AutonomousReturnSystem(double area_size, double area_size_delta);
	~AutonomousReturnSystem();
	void spinOnce();
private:
	void sendGoal();
	inline tf::StampedTransform getLastTF();
	ros::NodeHandle* node_;
	boost::shared_ptr<std_msgs::Int32 const> shared_ptr_;
	tf::TransformListener tf_listener;
};


AutonomousReturnSystem::AutonomousReturnSystem() :
	PoseVector<tf::StampedTransform>(), GoalController()
{
	node_ = new ros::NodeHandle("~");
}

AutonomousReturnSystem::AutonomousReturnSystem(double area_size, double area_size_delta) :
	PoseVector<tf::StampedTransform>(area_size, area_size_delta), GoalController()
{
	node_ = new ros::NodeHandle("~");
}

AutonomousReturnSystem::~AutonomousReturnSystem()
{
	node_->shutdown();
	delete(node_);
}

void AutonomousReturnSystem::spinOnce()
{
	//listen joystick state 
	actionlib::SimpleClientGoalState move_base_client_state = actionlib::SimpleClientGoalState::PENDING;
	if(GoalController::isStarted())
		move_base_client_state = move_base_client_->getState();
	shared_ptr_ = ros::topic::waitForMessage<std_msgs::Int32>("/teleop_verification", ros::Duration(1.0));
	if(shared_ptr_ != NULL)
	{
		// check if move_base is active
		//TODO need to check
		if(move_base_client_state == actionlib::SimpleClientGoalState::ACTIVE)
		{
			move_base_client_->cancelGoal();
			ROS_INFO_STREAM("Aborting current goal " << this->toString());
			return;
		}
		else
		{//listen for new transforms
			tf::StampedTransform cur_tf;
			try
			{
				cur_tf = getLastTF();
			}
			catch(tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
        		return;
			}
			this->smart_push_back(cur_tf);
		}
	}
	else //controller may be disconnected
	{
		if(!GoalController::isStarted()){
			GoalController::setStarted();
			ROS_INFO_STREAM("Starting GoalController. Sending first goal " << this->toString());
			sendGoal();
		}
		//TODO
		if(this->empty())
			return;
		if(move_base_client_state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO_STREAM("Succeded goal. ");
			try
			{
				this->safe_pop_back();
			}
			catch(std::exception& e)
			{
				ROS_INFO("%s",e.what());
			}
			ROS_INFO_STREAM("Sending new goal " << this->toString());
			sendGoal();
			return;
		}	
		if(move_base_client_state == actionlib::SimpleClientGoalState::ABORTED)
		{
			ROS_INFO("Goal aborting.");
			try
			{
				this->safe_pop_back();
			}
			catch(std::exception& e)
			{
				ROS_INFO("%s",e.what());
			}
			ROS_INFO_STREAM("Sending new goal " << this->toString());
			sendGoal();
			return;
		}
		if( move_base_client_state == actionlib::SimpleClientGoalState::PREEMPTED)
		{
			ROS_INFO_STREAM("Goal was preemted. Sending goal " << this->toString());
			sendGoal();
			return;
		}
		if( move_base_client_state ==  actionlib::SimpleClientGoalState::RECALLED)
		{
			ROS_INFO_STREAM("Goal was recalled. Sending goal " << this->toString());
			sendGoal();
			return;
		}
		return;
	}
}

//get last tranform from tf server
inline tf::StampedTransform AutonomousReturnSystem::getLastTF()
{
	tf::StampedTransform cur_tf;
	try
	{
    	tf_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
    	tf_listener.lookupTransform("/map", "/base_link",
        ros::Time(0), cur_tf);
    }
    catch (tf::TransformException ex)
    {
    	throw ex;
    }
    return cur_tf;
}



//send goal to action server
void AutonomousReturnSystem::sendGoal()
{
	if(!this->empty())
		GoalController::sendGoal(convert<tf::StampedTransform, move_base_msgs::MoveBaseGoal>(this->back()));
	else
		ROS_INFO("PoseVector is empty");
}

#endif
