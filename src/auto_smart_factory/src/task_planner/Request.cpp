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

Request::Request(TaskPlanner* tp, TaskRequirementsConstPtr taskRequirements, std::string type)
		:
		taskPlanner(tp), requirements(taskRequirements) {
	ROS_ASSERT(tp);
	ROS_ASSERT(taskRequirements);

	status.id = getNewId();
	status.type = type;
	status.create_time = ros::Time::now();
	status.status = "just created";
	status.pkg_config = taskRequirements->getPackageConfig();

	ros::NodeHandle pn("~");
	pn.param("use_best_eta", useBestETA, true);

	ROS_INFO("[request %d] Use best ETA: %s", getId(), std::to_string(useBestETA).c_str());
}

Request::~Request() {
}

unsigned int Request::getNewId() {
	return nextId++;
}

TaskData Request::allocateResources() {
	ROS_WARN("[request %d] Allocate resources...", status.id);

	// find candidate source tray(s)
	std::vector<Tray> sourceTrayCandidates;
	if(!findSourceCandidates(sourceTrayCandidates)) {
		status.status = "No source tray candidates available.";
		throw std::runtime_error(status.status);
	}

	ROS_WARN("[request %d] Found %ld source tray candidates.", status.id,
	         sourceTrayCandidates.size());

	// find candidate target tray(s)
	std::vector<Tray> targetTrayCandidates;
	if(!findTargetCandidates(targetTrayCandidates)) {
		status.status = "No target tray candidates available.";
		throw std::runtime_error(status.status);
	}

	ROS_WARN("[request %d] Found %ld target tray candidates.", status.id,
	         targetTrayCandidates.size());

	// find candidate robot(s)
	status.status = "getting candidates";
	if(!getRobotCandidates(sourceTrayCandidates,
	                       targetTrayCandidates)) {
		status.status = "No robot candidates available.";
		throw std::runtime_error(status.status);
	}

	status.status = "trying to allocate a candidate";
	ROS_WARN("[request %d] Found %ld robot candidates.", status.id,
	         robotCandidates.size());

	// try one robot after the other until success
	for(const RobotCandidate& cand : robotCandidates) {
		// allocate trays
		ROS_INFO("[request %d] for %s Source tray id is: %d and target tray id is %d", status.id, cand.robotId.c_str(), cand.source.id, cand.target.id);
		TrayAllocatorPtr sourceTray = TrayAllocator::allocateTray(
				cand.source.id);
		TrayAllocatorPtr targetTray = TrayAllocator::allocateTray(
				cand.target.id);

		if(sourceTray->isValid() && targetTray->isValid()) {
			ROS_WARN(
					"[request %d] Successfully allocated source %d and target %d.",
					status.id, cand.source.id, cand.target.id);

			// assure that source and target are still suitable
			if(!requirements->checkAllocatedSourceTray(cand.source)) {
				ROS_INFO("[request %d] Checking allocated source tray failed.", status.id);
				continue;
			}
			if(!requirements->checkAllocatedTargetTray(cand.target)) {
				ROS_INFO("[request %d] Checking allocated target tray failed.", status.id);
				continue;
			}

			ROS_WARN(
					"[request %d] Successfully checked robot ID %s allocated the source %d and the target %d.",
					status.id, cand.robotId.c_str(), cand.source.id, cand.target.id);

			// allocate robot (try to assign task)
			if(!allocateRobot(cand)) {
				ROS_WARN("[request %d] robot allocation fail", status.id);
				continue;
			}

			// copy package information
			Package pkg = sourceTray->getPackage();
			if(!targetTray->setPackage(pkg)) {
				ROS_ERROR("[request %d] Could not set package information at target tray (id: %d, type: %d)!",
				          status.id, pkg.id, pkg.type_id);
			} else {
				ROS_WARN("[request %d] Successfully set package at target tray (id: %d, type: %d)!", status.id, pkg.id,
				         pkg.type_id);
			}

			ROS_WARN("[request %d] Successfully allocated robot %s.", status.id,
			         cand.robotId.c_str());
			ROS_INFO(
					"[request %d] All resources were allocated successfully! Starting execution...",
					status.id);

			// successfully allocated all resources
			status.status = "allocated resources";
			return TaskData(cand, sourceTray, targetTray, pkg, status.create_time);
		}

		ROS_WARN("[request %d] Allocation failed.", status.id);
	}

	status.status = "All allocations failed.";
	throw std::runtime_error(status.status);
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

bool Request::findSourceCandidates(
		std::vector<auto_smart_factory::Tray>& sourceTrayCandidates) const {
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
			// In the early versions, below had to be done as there were no roadmaps drawn to these storages !!!
			// The code below can still be used in directing the robots to the certain trays for testing purposes.
			// if (tray.second.id != 13 && tray.second.id != 14 && tray.second.id != 17 && tray.second.id != 18){
			//	targetTrayCandidates.push_back(tray.second);
			//ROS_WARN("Pushed back candidate and size is %i", targetTrayCandidates.size());
			// }
			// else{
			//ROS_WARN("TRAY ID:%d has been deleted from the candidates since has no roadmap drawn!",tray.second.id);
			//}
		}
	}

	return !targetTrayCandidates.empty();
}

bool Request::allocateRobot(RobotCandidate candidate) const {
	ROS_INFO("In Request::allocateRobot");
	ros::NodeHandle n;
	ros::ServiceClient assignTaskClient = n.serviceClient<AssignTask>(
			"/" + candidate.robotId + "/assign_task");

	AssignTask srv;
	srv.request.task_id = status.id;
	srv.request.input_tray = candidate.source.id;
	srv.request.storage_tray = candidate.target.id;

	if(assignTaskClient.call(srv)) {
		ROS_INFO("[request %d] The task= %d was assigned at time= %f .",
		         status.id, status.id, ros::Time::now().toSec());

		ROS_FATAL("[request %d] was assigned to %s with Task score %.2f", status.id, candidate.robotId.c_str(), candidate.score);

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

void Request::receiveTaskResponse(const auto_smart_factory::TaskRating& tr){
	if(this->status.id != tr.request_id){
		ROS_INFO("[Request %d] received answer to another [Request %d] from %s", this->status.id, tr.request_id, tr.robot_id.c_str());
		return;
	}
	if(!tr.reject){
		// add robot as candidate
		RobotCandidate cand;
		cand.score = tr.score;
		cand.robotId = tr.robot_id;
		// ROS_INFO("[Request %d] looking for source tray %d and target tray %d", this->status.id, tr.start_id, tr.end_id);
		cand.source = taskPlanner->getTrayConfig(tr.start_id);
		cand.target = taskPlanner->getTrayConfig(tr.end_id);
		robotCandidates.push_back(cand);
	}
	answeredRobots[tr.robot_id] = tr.reject;
	ROS_INFO("[Request %d] still missing %d answers", this->status.id, (int)(taskPlanner->getRegisteredRobots().size() - answeredRobots.size()));
}

bool Request::getRobotCandidates(const std::vector<Tray>& sourceTrayCandidates,
                                 const std::vector<Tray>& targetTrayCandidates){

	robotCandidates.clear();
	answeredRobots.clear();

	taskPlanner->publishTask(sourceTrayCandidates, targetTrayCandidates, status.id);
	
	waitForRobotScores(ros::Duration(1), ros::Rate(10));

	std::sort(robotCandidates.begin(), robotCandidates.end(),
		          [](RobotCandidate first, RobotCandidate second) {
			          return first.score < second.score;
		          });

	return !robotCandidates.empty();
}

void Request::waitForRobotScores(ros::Duration timeout, ros::Rate frequency){
	ros::Time start = ros::Time::now();
	ros::Time end = start + timeout;
	ROS_INFO("[Request %d] is waiting for robot scores", status.id);
	ros::Time now = start;
	while(ros::Time::now() < end){
		if(taskPlanner->getRegisteredRobots().size() == answeredRobots.size()){
			ROS_INFO("[Request %d] received all answers", status.id);
			return;
		}
		ros::spinOnce();
		frequency.sleep();
	}
	// ROS_INFO("[Request %d] Timeout while waiting for robot scores, got %d answers", status.id, (unsigned int)answeredRobots.size());
	return;
}