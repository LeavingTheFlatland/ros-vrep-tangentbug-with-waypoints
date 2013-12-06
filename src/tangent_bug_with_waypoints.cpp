/*
* =====================================================================================
*
 *       Filename:  tangent_bug_with_waypoints.cpp
 *
 *    Description:  BUG2 algorithm implementation for ROS and VREP
 *
 *        Version:  1.0
 *        Created:  19/11/2013 15:59:17
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Roberto Marino (rm), formica@member.fsf.org
 *   Organization:  University of Genoa
 *
 * =====================================================================================
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <iterator>

using namespace std;

enum BugState {BEHAV_1, BEHAV_2};

class Bug2Vrep
{
	public:
		Bug2Vrep();
		enum BugState state;

		ros::Publisher Control_pub;

		ros::Publisher Motor1_pub;
		ros::Publisher Motor2_pub;
		ros::Publisher Motor3_pub;
		ros::Publisher Motor4_pub;

		ros::Publisher QuadTargetPosition_pub;

		pcl::PointCloud<pcl::PointXYZ> cloud;
		geometry_msgs::Point QuadPos;	// GET Variable
		geometry_msgs::Point GoalPos;
		geometry_msgs::Point ComputeMotionToGoal(void);
		geometry_msgs::Point MinimumDistanceToGoal(void);


	private:
		// Callbacks
                void RangeFinderCallback(const std_msgs::String::ConstPtr& msg);
                void RangeFinderPC2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
		void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
                void QuadPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		float ComputeDistance(geometry_msgs::Point x,geometry_msgs::Point y);
				
		ros::NodeHandle nh_;
                
		ros::Subscriber RangeFinder_sub;	// RangeFinder msgs from the robot
		ros::Subscriber RangeFinderPC2_sub;
                ros::Subscriber GoalPose_sub;		// Pose of the goal, useful to compute the distance
		ros::Subscriber QuadPose_sub;		// Pose of the robot from ground thruth or odometry 
			
				
		sensor_msgs::PointCloud2 RangeData;	// GET Variable
		geometry_msgs::PoseStamped GoalPose;	// GET Variable
		geometry_msgs::Point TargetPos; 	// SET Variable
};

Bug2Vrep::Bug2Vrep()
{
        RangeFinder_sub = nh_.subscribe<std_msgs::String>("/vrep/RangeFinderData",10, &Bug2Vrep::RangeFinderCallback, this); 
       	RangeFinderPC2_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/vrep/RangeFinderDataPC2",10,&Bug2Vrep::RangeFinderPC2Callback,this); 
	GoalPose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/vrep/GoalPose",10,&Bug2Vrep::GoalPoseCallback,this);
       	QuadPose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/vrep/QuadPose",10,&Bug2Vrep::QuadPoseCallback,this);
	Control_pub = nh_.advertise<std_msgs::String>("/vrep/QuadMotorControl",1);	

	Control_pub = nh_.advertise<std_msgs::String>("/vrep/QuadMotorControl",1);	
		
	QuadTargetPosition_pub = nh_.advertise<geometry_msgs::Point>("/vrep/QuadrotorWaypointControl",1);
	
	Motor1_pub = nh_.advertise<std_msgs::Float32>("/vrep/Motor1",1);
	Motor2_pub = nh_.advertise<std_msgs::Float32>("/vrep/Motor2",1);
	Motor3_pub = nh_.advertise<std_msgs::Float32>("/vrep/Motor3",1);
	Motor4_pub = nh_.advertise<std_msgs::Float32>("/vrep/Motor4",1);
}

void Bug2Vrep::RangeFinderPC2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*msg, cloud);
	for(int i=0;i<cloud.points.size(); ++i)
	{
		ROS_INFO("Point: %f %f %f", cloud.points[i].x,cloud.points[i].y,cloud.points[i].z);
	}
}


void Bug2Vrep::RangeFinderCallback(const std_msgs::String::ConstPtr& msg)
{
        //ROS_INFO("RangeFinderData: %s", msg->data.c_str());
}

void Bug2Vrep::GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
        GoalPos.x = msg->pose.position.x;
	GoalPos.y = msg->pose.position.y;
	GoalPos.z = msg->pose.position.z;
}

void Bug2Vrep::QuadPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	QuadPos.x = msg->pose.position.x;
	QuadPos.y = msg->pose.position.y;
	QuadPos.z = msg->pose.position.z;       
}

geometry_msgs::Point Bug2Vrep::ComputeMotionToGoal(void)
{
	float theta;
	geometry_msgs::Point delta;

	theta = atan2((GoalPos.y - QuadPos.y),(GoalPos.x - QuadPos.x));
	delta.x = 0.2*(cos(theta));
	delta.y = 0.2*(sin(theta));
	return delta;	
}

float Bug2Vrep::ComputeDistance(geometry_msgs::Point a, geometry_msgs::Point b)
{
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

geometry_msgs::Point Bug2Vrep::MinimumDistanceToGoal(void)
{
	int n = cloud.points.size();
	vector<float> distances(n);
	geometry_msgs::Point pnt[n];

	for(int i=0;i<n; ++i)
	{
		pnt[i].x = cloud.points[i].x;
		pnt[i].y = cloud.points[i].y;
		distances[i] = Bug2Vrep::ComputeDistance(QuadPos,pnt[i]) + Bug2Vrep::ComputeDistance(pnt[i],GoalPos);
	}
	
//	return pnt[*std::min_element(distances,distances+n)];
	int min_pos = distance(distances.begin(),min_element(distances.begin(), distances.end()));
	return pnt[min_pos];
}


int main(int argc, char** argv)
{
	float m1, m2, m3, m4;
        ros::init(argc,argv, "Bug2Vrep");
        Bug2Vrep bvrep;
  
	geometry_msgs::Point TargetPosition;
	geometry_msgs::Point Delta;
	
	while(ros::ok())
	{	
		ros::spinOnce();
		
		TargetPosition.x = bvrep.QuadPos.x;
		TargetPosition.y = bvrep.QuadPos.y;
		TargetPosition.z = bvrep.QuadPos.z;

		ROS_INFO("[QuadPosition]: %f %f %f",bvrep.QuadPos.x,bvrep.QuadPos.y,bvrep.QuadPos.z);
		
		Delta = bvrep.ComputeMotionToGoal();		
		
		//--- Navigation algorithm HERE
		TargetPosition.x += Delta.x;
		TargetPosition.y += Delta.y;
		


		//--- END Navigation algorithm
		bvrep.QuadTargetPosition_pub.publish(TargetPosition);		// Send the control signal
		
		usleep(5000000);
	}
	ros::shutdown();
	printf("TangentBug Algorithm ended!");

	return(0);
}


