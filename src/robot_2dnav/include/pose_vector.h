#ifndef POSE_VECTOR_H_
#define POSE_VECTOR_H_

#include <vector>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>

class PopFromEmptyPoseVectorExeption: public std::exception
{
public:
	virtual const char* what() const throw()
	{
		return "Can't pop the element from PoseVector. PoseVector is empty";
	}
};


template<class T>
class PoseVector: public std::vector<T>
{
public:
	PoseVector();
	PoseVector(double area_size, double area_size_delta);
	~PoseVector();
	void safe_pop_back();
	void smart_push_back(const T& pose);
	std::string toString(const T& pose);
	std::string toString();
private:
	double area_size_;
	double area_size_delta_;
	double max_area_size_; 
};


template<class T>
inline double getDistance(const T& pose_1, const T& pose_2);

template<class O, class T>
inline T convert(const O& obj);


template<class T>
PoseVector<T>::PoseVector():
	std::vector<T>(),
	area_size_(3.0),
	area_size_delta_(0.2),
	max_area_size_(area_size_delta_)
{

}
	
template<class T>
PoseVector<T>::PoseVector(double area_size, double area_size_delta) :  
	std::vector<T>(),
	area_size_(area_size), 
	area_size_delta_(area_size_delta),
	max_area_size_(area_size + area_size_delta)
{ 

}

template<class T>
PoseVector<T>::~PoseVector()
{

}

template<class T>
void PoseVector<T>::safe_pop_back()
{
	if(!this->empty())
	{
		ROS_INFO_STREAM("Deleting last pose " << toString());
		this->pop_back();
	}
	else
		throw new PopFromEmptyPoseVectorExeption();
}
/*
template<>
double getDistance<move_base_msgs::MoveBaseGoal>(const move_base_msgs::MoveBaseGoal& pose_1, const move_base_msgs::MoveBaseGoal& pose_2)
{
	return sqrt(
		(pose_1.target_pose.pose.position.x - pose_2.target_pose.pose.position.x)*(pose_1.target_pose.pose.position.x - pose_2.target_pose.pose.position.x)
		+
        (pose_1.target_pose.pose.position.y - pose_2.target_pose.pose.position.y)*(pose_1.target_pose.pose.position.y - pose_2.target_pose.pose.position.y)
        +
        (pose_1.target_pose.pose.position.z - pose_2.target_pose.pose.position.z)*(pose_1.target_pose.pose.position.z - pose_2.target_pose.pose.position.z)
        );
}
*/

template<>
inline double getDistance<tf::StampedTransform>(const tf::StampedTransform& tf1, const tf::StampedTransform& tf2)
{
  return sqrt((tf1.getOrigin().x() - tf2.getOrigin().x())*(tf1.getOrigin().x() - tf2.getOrigin().x())+
              (tf1.getOrigin().y() - tf2.getOrigin().y())*(tf1.getOrigin().y() - tf2.getOrigin().y())+
              (tf1.getOrigin().z() - tf2.getOrigin().z())*(tf1.getOrigin().z() - tf2.getOrigin().z())
              );
}

template<>
inline double getDistance<geometry_msgs::Point>(const geometry_msgs::Point& p_1, const geometry_msgs::Point& p_2)
{
	return sqrt((p_1.x - p_2.x)*(p_1.x - p_2.x) +
				(p_1.y - p_2.y)*(p_1.y - p_2.y) +
				(p_1.z - p_2.z)*(p_1.x - p_2.z));
}

template<>
inline geometry_msgs::Point convert<tf::StampedTransform, geometry_msgs::Point>(const tf::StampedTransform& pose)
{
	geometry_msgs::Point p;
	p.x = pose.getOrigin().x();
	p.y = pose.getOrigin().y();
	p.z = pose.getOrigin().z();
	return p;
}

template<>
inline move_base_msgs::MoveBaseGoal convert<tf::StampedTransform, move_base_msgs::MoveBaseGoal>(const tf::StampedTransform& pose)
{
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "/map";
  	goal.target_pose.header.stamp = ros::Time(0);
	goal.target_pose.pose.position.x = pose.getOrigin().x();
    goal.target_pose.pose.position.y = pose.getOrigin().y();
    goal.target_pose.pose.position.z = pose.getOrigin().z();
    goal.target_pose.pose.orientation.w = 1.0;
    return goal;
}

//convert to string pose
template<class T>
std::string PoseVector<T>::toString(const T& pose)
{
	geometry_msgs::Point p = convert<T, geometry_msgs::Point>(pose);
	std::stringstream ss;
	ss << "x = " << p.x << "y =" << p.y << "z = " << p.z;
	return ss.str();
}

//convert to string last pose of PoseVector 
template<class T>
std::string PoseVector<T>::toString()
{
	geometry_msgs::Point p = convert<T, geometry_msgs::Point>(this->back());
	std::stringstream ss;
	ss << "x = " << p.x << "y =" << p.y << "z = " << p.z;
	return ss.str();
}

template<class T>
void PoseVector<T>::smart_push_back(const T& pose)
{
	if(this->empty())
	{
		this->push_back(pose);
		ROS_INFO_STREAM("The PoseVector was empty. Added pose " << toString(pose));
		return;
	}
	//if pose_vector size == 1 then
	double dist_1 = getDistance<T>(pose, this->back());
	if(this->size() == 1 && dist_1 >= area_size_)
	{
		this->push_back(pose);
		ROS_INFO_STREAM("Added new pose " << toString(pose));
		return;
	}
	//if pose_vector size >= 2 we need to consider last but one pose
	if(this->size() >=2){
		double dist_2 = getDistance<T>(pose, this->at(this->size()-2));
		if(dist_1 >= area_size_ && dist_2 >= max_area_size_)
		{
			this->push_back(pose);
			ROS_INFO_STREAM("Added new pose " << toString(pose));
			return;
		}
		if(dist_1 >= area_size_ && dist_2 <= max_area_size_ && dist_2 >= area_size_)
		{
			this->back() = pose;
			ROS_INFO_STREAM("Current pose is closer than last pose. Replaced with pose " << toString(pose));
			return;
		}
		if(dist_2 <= area_size_)
		{
			ROS_INFO_STREAM("Current pose is closer than last pose.");
			safe_pop_back();	
			return;
		}
	}
	return;
}

#endif