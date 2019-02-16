/*
 * Task.h
 *
 *  Created on: 09.07.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASK_H_
#define AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASK_H_

#include <thread>

#include "auto_smart_factory/TaskState.h"
#include "auto_smart_factory/TaskStarted.h"
#include "auto_smart_factory/GetStorageState.h"
#include "auto_smart_factory/AssignTask.h"
#include "auto_smart_factory/RobotConfiguration.h"
#include "auto_smart_factory/StorageUpdate.h"
#include "auto_smart_factory/GripperState.h"
#include "auto_smart_factory/ReserveStorageTray.h"
#include "task_planner/RobotCandidate.h"
#include "task_planner/TaskRequirements.h"
#include "task_planner/TaskData.h"
#include "task_planner/InputTaskRequirements.h"
#include "task_planner/OutputTaskRequirements.h"

class TaskPlanner;

/**
 * This class represents a started task.
 * It implements the supervision process and resource releasing.
 */
class Task {
public:
	/**
	 * Start a new task using allocated resources.
	 * @param id Task id (inherited from request id)
	 * @param taskData Allocated resources and other task information
	 */
	Task(unsigned int id, TaskData taskData);

	virtual ~Task() = default;

	/**
	 * Execute this task. This created a new thread which then starts
	 * task work (supervision).
	 */
	void execute();

	/**
	 * Returns task id
	 * @return Task id
	 */
	unsigned int getId() const;

	/**
	 * Returns task state information.
	 * @return State information
	 */
	const auto_smart_factory::TaskState& getState() const;

	/**
	 * Checks if the associated thread can be joined.
	 * This is only possible if the task was finished.
	 * @return True if joinable
	 */
	bool isJoinable() const;

	/**
	 * Joins the task thread. If thread is not already joinable, this blocks
	 * until task is finished.
	 */
	void join();

protected:
	/**
	 * This method is executed first by the task thread.
	 */
	void run();

	/**
	 * This method implements the task supervision process.
	 * @return True if supervision was successful
	 */
	bool superviseExecution();

	/**
	 * Busy wait for acknowledgement that task execution has started
	 * This function has no timeout as tasks can be queued to be executed an unkown
	 * amount of time in the future 
	 */
	void waitForTaskStartedAck();
	
	/**
	 * Busy wait for a load acknowledgment by tray sensor and robot.
	 *
	 * \todo Implement timeout detection based on estimated time
	 *
	 * @return True if acknowledgments were received before timeout
	 */
	bool waitForLoadAck();

	/**
	 * Busy wait for an unload acknowledgment by tray sensor and robot.
	 *
	 * \todo Implement timeout detection based on estimated time
	 *
	 * @return True if acknowledgments were received before timeout
	 */
	bool waitForUnloadAck();

	/**
	 * Used to receive if a task has started
	 * @param msg task started
	 */
	void receiveTaskStarted(const auto_smart_factory::TaskStarted& msg);

	/**
	 * Used to receive storage updates.
	 * @param msg Storage update
	 */
	void receiveLoadStorageUpdate(const auto_smart_factory::StorageUpdate& msg);

	/**
	 * Used to receive storage updates.
	 * @param msg Storage update
	 */
	void receiveUnloadStorageUpdate(const auto_smart_factory::StorageUpdate& msg);

	/**
	 * Receive robot gripper updates
	 * @param msg Robot gripper update
	 */
	void receiveRobotGripperUpdate(const auto_smart_factory::GripperState& msg);

protected:
	/// the task state
	auto_smart_factory::TaskState state;

	/// task data defining the task
	TaskData taskData;

	/// this thread is used to execute the task
	std::thread taskExecutionThread;

	bool loadAck, unloadAck;
	bool robotGrabAck, robotReleaseAck;
	bool taskStarted;
};

typedef std::shared_ptr<Task> TaskPtr;

#endif /* AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASK_H_ */
