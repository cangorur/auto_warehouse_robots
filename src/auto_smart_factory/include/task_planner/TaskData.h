/*
 * TaskData.h
 *
 *  Created on: 10.07.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_INCLUDE_TASK_PLANNER_TASKDATA_H_
#define AUTO_SMART_FACTORY_INCLUDE_TASK_PLANNER_TASKDATA_H_

#include "ros/ros.h"

#include "auto_smart_factory/Tray.h"
#include "auto_smart_factory/Package.h"
#include "task_planner/RobotCandidate.h"
#include "storage_management/TrayAllocator.h"

/**
 * This class comprises all information that needs to be passed from a
 * request that successfully allocated all resources to a newly created
 * task.
 */
class TaskData {
public:
	TaskData(const RobotCandidate& candidate, TrayAllocatorPtr allocatedSource, TrayAllocatorPtr allocatedTarget, auto_smart_factory::Package pkg, ros::Time createTime);

	virtual ~TaskData() = default;

	/// The response of the robot containing robot id, estimated duration, source and target trays
	RobotCandidate robotOffer;

	/// Allocated source tray
	TrayAllocatorPtr allocatedSource;

	/// Allocated target tray
	TrayAllocatorPtr allocatedTarget;

	/// Package contained in the source tray
	auto_smart_factory::Package package;

	/// Time stamp when the associated request was created
	ros::Time createTime;
};

#endif /* AUTO_SMART_FACTORY_INCLUDE_TASK_PLANNER_TASKDATA_H_ */
