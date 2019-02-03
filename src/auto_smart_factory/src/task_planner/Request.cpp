/*
 * Request.cpp
 *
 *  Created on: 21.07.2017
 *      Author: jacob
 */

#include <task_planner/Request.h>
#include <task_planner/TaskPlanner.h>
#include <auto_smart_factory/GetTrayState.h>
#include <auto_smart_factory/StorePackage.h>
#include <auto_smart_factory/RetrievePackage.h>

#include <algorithm>

using namespace auto_smart_factory;

unsigned int Request::nextId = 0;

Request::Request(TaskPlanner* tp, TaskRequirementsConstPtr taskRequirements, std::string type) :
		taskPlanner(tp),
		requirements(taskRequirements) {
	ROS_ASSERT(tp);
	ROS_ASSERT(taskRequirements);

	status.id = getNewId();
	status.type = type;
	status.create_time = ros::Time::now();
	this->status.status = "just created";
	status.pkg_config = taskRequirements->getPackageConfig();

	ros::NodeHandle pn("~");
	pn.param("use_best_eta", useBestETA, true);

	ROS_INFO("[request %d] Use best ETA: %s", getId(), std::to_string(useBestETA).c_str());
}

unsigned int Request::getNewId() {
	return nextId++;
}

TaskData Request::allocateResources() {
	// ROS_INFO("[request %d] Allocate resources...", status.id);

	// find candidate source tray(s)
	std::vector<Tray> sourceTrayCandidates;
	if(!findSourceCandidates(sourceTrayCandidates)) {
		this->status.status = "No source tray candidates available.";
		throw std::runtime_error(this->status.status);
	}

	// ROS_INFO("[request %d] Found %ld source tray candidates.", status.id, sourceTrayCandidates.size());

	// find candidate target tray(s)
	std::vector<Tray> targetTrayCandidates;
	if(!findTargetCandidates(targetTrayCandidates)) {
		this->status.status = "No target tray candidates available.";
		throw std::runtime_error(this->status.status);
	}

	// ROS_INFO("[request %d] Found %ld target tray candidates.", status.id, targetTrayCandidates.size());

	// find candidate robot(s)
	this->status.status = "getting candidates";
	if(!getRobotCandidates(sourceTrayCandidates, targetTrayCandidates)) {
		this->status.status = "No robot candidates available.";
		throw std::runtime_error(this->status.status);
	}

	this->status.status = "trying to allocate a candidate";
	// ROS_INFO("[request %d] Found %ld robot candidates.", status.id, robotCandidates.size());

	// try one robot after the other until success
	for(const RobotCandidate& candidate : robotCandidates) {
		// ROS_INFO("[request %d] Allocating robots for %s Source tray id is: %d and target tray id is %d", status.id, candidate->robotId.c_str(), candidate->source.id, candidate->target.id);
		TrayAllocatorPtr sourceTray = TrayAllocator::allocateTray(candidate.source.id);
		TrayAllocatorPtr targetTray = TrayAllocator::allocateTray(candidate.target.id);

		if(sourceTray->isValid() && targetTray->isValid()) {
			// ROS_INFO("[request %d] Successfully allocated source %d and target %d.", status.id, candidate->source.id, candidate->target.id);

			// assure that source and target are still suitable
			if(!requirements->checkAllocatedSourceTray(candidate.source)) {
				//ROS_INFO("[request %d] Checking allocated source tray failed.", this->status.id);
				continue;
			}
			if(!requirements->checkAllocatedTargetTray(candidate.target)) {
				//ROS_INFO("[request %d] Checking allocated target tray failed.", this->status.id);
				continue;
			}

			// allocate robot (try to assign task)
			if(!allocateRobot(candidate)) {
				ROS_WARN("[request %d] robot allocation fail", this->status.id);
				continue;
			}

			// copy package information
			Package pkg = sourceTray->getPackage();
			if(!targetTray->setPackage(pkg)) {
				ROS_ERROR("[request %d] Could not set package information at target tray (id: %d, type: %d)!", this->status.id, pkg.id, pkg.type_id);
			} else {
				ROS_INFO("[request %d] Successfully set package at target tray (id: %d, type: %d)!", this->status.id, pkg.id, pkg.type_id);
			}

			ROS_INFO("[request %d] Successfully allocated robot %s.", this->status.id, candidate.robotId.c_str());
			//ROS_INFO("[request %d] All resources were allocated successfully! Starting execution...", this->status.id);

			// successfully allocated all resources
			this->status.status = "allocated resources";
			return TaskData(candidate, sourceTray, targetTray, pkg, this->status.create_time);
		}

		ROS_WARN("[request %d] Allocation failed.", this->status.id);
	}

	this->status.status = "All allocations failed.";
	throw std::runtime_error(this->status.status);
}

bool Request::isPending() const {
	if(status.type == "input") {
		// check if input is still occupied and not reserved
		Tray inputTray = taskPlanner->getTrayConfig(requirements->getKnownTrayId());
		return requirements->isLegalSourceTray(inputTray);
	}
	if(status.type == "output") {
		// output request can only be satisfied by running the task
		return true;
	}

	throw std::runtime_error("Request type has invalid value!");
}

bool Request::findSourceCandidates(std::vector<auto_smart_factory::Tray>& sourceTrayCandidates) const {
	sourceTrayCandidates.clear();

	for(auto const& tray : taskPlanner->getTrayConfigs()) {
		if(requirements->isLegalSourceTray(tray.second)) {
			sourceTrayCandidates.push_back(tray.second);
		}
	}

	return !sourceTrayCandidates.empty();
}

bool Request::findTargetCandidates(std::vector<auto_smart_factory::Tray>& targetTrayCandidates) const {
	targetTrayCandidates.clear();

	for(auto const& tray : taskPlanner->getTrayConfigs()) {
		if(requirements->isLegalTargetTray(tray.second)) {
			targetTrayCandidates.push_back(tray.second);
		}
	}

	return !targetTrayCandidates.empty();
}

bool Request::allocateRobot(const RobotCandidate& candidate) const {
	ros::NodeHandle n;
	ros::ServiceClient assignTaskClient = n.serviceClient<AssignTask>("/" + candidate.robotId + "/assign_task");

	AssignTask srv;
	srv.request.task_id = status.id;
	srv.request.input_tray = candidate.source.id;
	srv.request.storage_tray = candidate.target.id;

	if(assignTaskClient.call(srv)) {
		ROS_INFO("[request %d] was assigned to %s with Task score %.2f", status.id, candidate.robotId.c_str(), candidate.score);
		return srv.response.success;
	}

	return false;
}

unsigned int Request::getId() const {
	return status.id;
}

auto_smart_factory::RequestStatus Request::getStatus() const {
	return status;
}

TaskRequirementsConstPtr Request::getRequirements() const {
	return requirements;
}

void Request::receiveTaskResponse(const auto_smart_factory::TaskRating& tr) {
	if(this->status.id != tr.request_id){
		ROS_INFO("[Request %d] received answer to another [Request %d] from %s", this->status.id, tr.request_id, tr.robot_id.c_str());
		return;
	}
	if(!tr.reject){
		// add robot as candidate
		ROS_ASSERT_MSG(tr.estimatedDuration > 0, "Tried to create robot candidate with estimatedDuration == 0");
		RobotCandidate candidate = RobotCandidate(tr.robot_id, taskPlanner->getTrayConfig(tr.start_id), taskPlanner->getTrayConfig(tr.end_id), tr.estimatedDuration, tr.score);
		robotCandidates.push_back(candidate);
	}
	
	// Use id (string) as key 
	answeredRobots[tr.robot_id] = tr.reject;
}

bool Request::getRobotCandidates(const std::vector<Tray>& sourceTrayCandidates, const std::vector<Tray>& targetTrayCandidates) {
	robotCandidates.clear();
	answeredRobots.clear();

	taskPlanner->publishTask(sourceTrayCandidates, targetTrayCandidates, status.id);
	
	waitForRobotScores(ros::Duration(1), ros::Rate(10));

	std::sort(robotCandidates.begin(), robotCandidates.end(),
		          [](const RobotCandidate& first, const RobotCandidate& second) {
			          return first.score < second.score;
		          });

	// ROS_INFO("[Request %d] finished getting candidates!", this->status.id);

	return !robotCandidates.empty();
}

void Request::waitForRobotScores(ros::Duration timeout, ros::Rate frequency){
	ros::Time start = ros::Time::now();
	ros::Time end = start + timeout;
	// ROS_INFO("[Request %d] is waiting for robot scores", status.id);
	while(ros::Time::now() < end){
		if(taskPlanner->getRegisteredRobots().size() == answeredRobots.size()){
			ROS_INFO("[Request %d] received all answers", status.id);
			return;
		}
		ros::spinOnce();
		frequency.sleep();
	}
	ROS_WARN("[Request %d] Timeout while waiting for robot scores, got %d scores", status.id, (unsigned int)answeredRobots.size());
}