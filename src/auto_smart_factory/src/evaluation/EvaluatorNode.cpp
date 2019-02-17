/*
 * EvaluatorNode.cpp
 *
 *  Created on: 13.02.2019
 *      Author: Florian Z.
 */

#include "ros/ros.h"
#include "evaluation/Evaluator.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "evaluator");
	ros::NodeHandle nh;

	Evaluator eval;
	ROS_INFO("Evaluator ready!");

	ros::Rate r(20);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
}