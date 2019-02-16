/*
 * Task.cpp
 *
 *  Created on: 09.07.2017
 *      Author: jacob
 */

#include "task_planner/Task.h"
#include "task_planner/TaskPlanner.h"
#include "auto_smart_factory/StorePackage.h"
#include "auto_smart_factory/RetrievePackage.h"
#include "auto_smart_factory/TaskStarted.h"

using namespace auto_smart_factory;

Task::Task(unsigned int id, TaskData taskData) :
		taskData(taskData) {
	// init task state
	state.id = id;
	state.status = "initialized";
	state.runTime.fromSec(0);
	state.loadTime.fromSec(0);
	state.unloadTime.fromSec(0);

	// copy fixed task data
	state.robot = taskData.robotOffer.robotId;
	state.package = taskData.package;
	state.sourceTray = taskData.robotOffer.source.id;
	state.targetTray = taskData.robotOffer.target.id;
	state.requestCreateTime = taskData.createTime;
	state.estimatedDuration = ros::Duration(taskData.robotOffer.estimatedDuration);

	ROS_INFO("[task %d] New task created: Robot %s carries package %d from tray %d to tray %d. Estimated duration is: %f sec", getId(), state.robot.c_str(), state.package.id, state.sourceTray, state.targetTray, state.estimatedDuration.toSec());
}

unsigned int Task::getId() const {
	return state.id;
}

const auto_smart_factory::TaskState& Task::getState() const {
	return state;
}

void Task::execute() {
	// run supervision in new thread
	taskExecutionThread = std::thread(&Task::run, this);
}

void Task::run() {
	// set run time
	state.runTime = ros::Time::now();

	state.status = "running supervision";

	// run supervision
	bool supervisionResult = superviseExecution();

	// TODO communicate result

	state.status = "finished";
}

bool Task::superviseExecution() {
	//ROS_INFO("[task %d] Start execution supervision...", getId());

	// supervise robot using TaskData
	// this should be executed in a separate task since it includes waiting
	state.status = "Waiting for starting acknowledgement";
	waitForTaskStartedAck();

	state.status = "Waiting for load acknowledgment.";

	// wait for unload acknowledgment
	if(!waitForLoadAck()) {
		ROS_ERROR("[task %d] Load acknowledgment timeout.", getId());
		state.status = "Waiting for load acknowledgment exceeded timeout.";
		return false;
	}

	// set load ack time
	state.loadTime = ros::Time::now();

	state.status = "Load acknowledged. Waiting for unload acknowledgment.";

	//ROS_INFO("[task %d] Received load acknowledgment.", getId());

	// clear package info and release allocated source tray after safety duration
	ros::Duration(1.0).sleep();
	taskData.allocatedSource->setPackage(Package());
	taskData.allocatedSource = nullptr;

	//ROS_INFO("[task %d] Released source tray.", getId());

	// wait for load acknowledgment
	if(!waitForUnloadAck()) {
		ROS_ERROR("[task %d] Unload acknowledgment timeout.", getId());
		state.status = "Waiting for unload acknowledgment exceeded timeout.";
		return false;
	}

	// set unload ack time
	state.unloadTime = ros::Time::now();

	state.status = "Unload acknowledged.";

	//ROS_INFO("[task %d] Received unload acknowledgment.", getId());

	// release allocated target tray after safety duration
	ros::Duration(1.0).sleep();
	taskData.allocatedTarget = nullptr;

	//ROS_INFO("[task %d] Released target tray.", getId());

	return true;
}

void Task::receiveTaskStarted(const auto_smart_factory::TaskStarted& msg){
	if(msg.started && getId() == msg.taskId){
		taskStarted = true;
	}
}

void Task::receiveLoadStorageUpdate(const StorageUpdate& msg) {
	if(msg.state.id == taskData.robotOffer.source.id && msg.action == StorageUpdate::DEOCCUPATION) {
		loadAck = true;
		// ROS_INFO("[task %d] Received tray load ack from tray %d.", getId(), msg.state.id);
	} else if(msg.state.id == taskData.robotOffer.source.id && msg.action == StorageUpdate::OCCUPATION) {
		loadAck = false;
		ROS_WARN("[task %d] Package that should be removed from tray %d was again put into it.", getId(), msg.state.id);
	}
}

void Task::receiveUnloadStorageUpdate(const StorageUpdate& msg) {
	if(msg.state.id == taskData.robotOffer.target.id && msg.action == StorageUpdate::OCCUPATION) {
		unloadAck = true;
		// ROS_INFO("[task %d] Received tray unload ack from tray %d.", getId(), msg.state.id);
	} else if(msg.state.id == taskData.robotOffer.target.id && msg.action == StorageUpdate::DEOCCUPATION) {
		unloadAck = false;
		ROS_WARN("[task %d] Package that should be put into tray %d was again removed from it.", getId(), msg.state.id);
	}
}

void Task::receiveRobotGripperUpdate(const auto_smart_factory::GripperState& msg) {
	if(msg.loaded) {
		robotGrabAck = true;
		// ROS_INFO("[task %d] Received gripper grab ack from robot.", getId());

		if(msg.package.id != taskData.package.id || msg.package.type_id != taskData.package.type_id) {
			ROS_ERROR("[task %d] Robot %s grabbed package from tray %d is not the package this task got assigned! (assigned package id=%d type=%d, grabbed package id=%d type=%d)", getId(), taskData.robotOffer.robotId.c_str(), taskData.allocatedSource->getId(), taskData.package.id, taskData.package.type_id, msg.package.id, msg.package.type_id);
		}
	} else {
		robotReleaseAck = true;
		// ROS_INFO("[task %d] Received gripper release ack from robot.", getId());

		if(msg.package.id != taskData.package.id || msg.package.type_id != taskData.package.type_id) {
			ROS_ERROR("[task %d] Robot %s released package to tray %d is not the package this task got assigned! (assigned package id=%d type=%d, released package id=%d type=%d)", getId(), taskData.robotOffer.robotId.c_str(), taskData.allocatedTarget->getId(), taskData.package.id, taskData.package.type_id, msg.package.id, msg.package.type_id);
		}
	}
}

void Task::waitForTaskStartedAck() {
	ros::NodeHandle n;
	taskStarted = false;

	ros::Subscriber startedTaskSub = n.subscribe("/" + taskData.robotOffer.robotId + "/task_started", 1000, &Task::receiveTaskStarted, this);
	ros::Rate r(5);
	while(!taskStarted){
		r.sleep();
	}
}

bool Task::waitForLoadAck() {
	ros::NodeHandle n;
	loadAck = false;
	robotGrabAck = false;

	ros::Subscriber storageUpdateSub = n.subscribe("/storage_management/storage_update", 1000, &Task::receiveLoadStorageUpdate, this);
	ros::Subscriber robotGripperUpdateSub = n.subscribe("/" + taskData.robotOffer.robotId + "/gripper_state", 1000, &Task::receiveRobotGripperUpdate, this);

	ros::Rate r(5);
	while(!(loadAck && robotGrabAck)) {
		// TODO: timeout?
		if(false) {
			return false;
		}

		r.sleep();
	}

	return true;
}

bool Task::waitForUnloadAck() {
	ros::NodeHandle n;
	unloadAck = false;
	robotReleaseAck = false;

	ros::Subscriber storageUpdateSub = n.subscribe("/storage_management/storage_update", 1000, &Task::receiveUnloadStorageUpdate, this);
	ros::Subscriber robotGripperUpdateSub = n.subscribe("/" + taskData.robotOffer.robotId + "/gripper_state", 1000, &Task::receiveRobotGripperUpdate, this);

	ros::Rate r(5);
	while(!(unloadAck && robotReleaseAck)) {
		// TODO: timeout?
		if(false) {
			return false;
		}

		r.sleep();
	}

	return true;
}

bool Task::isJoinable() const {
	return taskExecutionThread.joinable();
}

void Task::join() {
	taskExecutionThread.join();
}
