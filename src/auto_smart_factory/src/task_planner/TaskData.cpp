/*
 * TaskData.cpp
 *
 *  Created on: 10.07.2017
 *      Author: jacob
 */

#include "task_planner/TaskData.h"

TaskData::TaskData(const RobotCandidate& candidate, TrayAllocatorPtr allocatedSource, TrayAllocatorPtr allocatedTarget, auto_smart_factory::Package pkg, ros::Time createTime) :
		robotOffer(candidate),
		allocatedSource(allocatedSource),
		allocatedTarget(allocatedTarget),
		package(pkg),
		createTime(createTime)
{
	ROS_ASSERT(candidate.source.id == allocatedSource->getId());
	ROS_ASSERT(candidate.target.id == allocatedTarget->getId());
	ROS_ASSERT(candidate.estimatedDuration > 0);
}

