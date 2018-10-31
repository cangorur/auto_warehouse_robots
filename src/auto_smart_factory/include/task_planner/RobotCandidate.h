/*
 * RobotCandidate.h
 *
 *  Created on: 10.07.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_TASK_PLANNER_ROBOTCANDIDATE_H_
#define AUTO_SMART_FACTORY_SRC_TASK_PLANNER_ROBOTCANDIDATE_H_

#include <ros/ros.h>
#include <string>
#include <auto_smart_factory/Tray.h>

/**
 * This class encapsulates the response of a robot to a task request.
 * The robot selects source tray and target tray from the candidate lists
 * and computes an estimated duration.
 */
class RobotCandidate {
public:
	RobotCandidate();

	/// Robot id
	std::string robotId;

	/// Selected source tray
	auto_smart_factory::Tray source;

	/// Selected target tray
	auto_smart_factory::Tray target;

	/// Estimated task duration
	ros::Duration estimatedDuration;
};

#endif /* AUTO_SMART_FACTORY_SRC_TASK_PLANNER_ROBOTCANDIDATE_H_ */
