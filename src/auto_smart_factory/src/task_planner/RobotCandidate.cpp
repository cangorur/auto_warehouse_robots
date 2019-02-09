#include <utility>

/*
 * RobotCandidate.cpp
 *
 *  Created on: 10.07.2017
 *      Author: jacob
 */

#include "task_planner/RobotCandidate.h"

RobotCandidate::RobotCandidate(std::string robotId, auto_smart_factory::Tray source, auto_smart_factory::Tray target, double estimatedDuration, double score) :
	robotId(std::move(robotId)),
	source(std::move(source)),
	target(std::move(target)),
	estimatedDuration(estimatedDuration),
	score(score)
{
	ROS_ASSERT_MSG(estimatedDuration > 0, "estimatedDuration: %f", estimatedDuration);
}
