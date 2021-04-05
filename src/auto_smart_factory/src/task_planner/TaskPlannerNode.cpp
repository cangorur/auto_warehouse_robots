/*
 * TaskPlannerNode.cpp
 *
 *  Created on: 11.07.2017
 *      Author: jacob
 */

#include "ros/ros.h"
#include "task_planner/TaskPlanner.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "task_planner");
	ros::NodeHandle nh;

	TaskPlanner taskPlanner;
	//ROS_INFO("Task planner ready!");

	ros::Rate r(10);
	while(ros::ok()) {
		taskPlanner.update();
		ros::spinOnce();
		r.sleep();
	}
}
