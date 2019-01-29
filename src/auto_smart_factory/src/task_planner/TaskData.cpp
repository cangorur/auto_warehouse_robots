/*
 * TaskData.cpp
 *
 *  Created on: 10.07.2017
 *      Author: jacob
 */

#include <task_planner/TaskData.h>

TaskData::TaskData(const RobotCandidate& cand, TrayAllocatorPtr allocatedSource,
                   TrayAllocatorPtr allocatedTarget, auto_smart_factory::Package pkg, ros::Time createTime)
		:
		robotOffer(cand), allocatedSource(allocatedSource), allocatedTarget(
		allocatedTarget), package(pkg), createTime(createTime) {
	ROS_ASSERT(cand.source.id == allocatedSource->getId());
	ROS_ASSERT(cand.target.id == allocatedTarget->getId());
}

TaskData::~TaskData() {
}

