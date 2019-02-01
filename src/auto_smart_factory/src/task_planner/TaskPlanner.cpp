/*
 * TaskPlanner.cpp
 *
 *  Created on: 09.07.2017
 *      Author: jacob
 */

#include <task_planner/TaskPlanner.h>

using namespace auto_smart_factory;

TaskPlanner::TaskPlanner() {
	ros::NodeHandle pn("~");
	initServer = pn.advertiseService("init", &TaskPlanner::initialize, this);

	ROS_INFO("Task planner created...");
}

bool TaskPlanner::initialize(InitTaskPlannerRequest& req, InitTaskPlannerResponse& res) {
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	// build package config map
	for(auto config : req.package_configurations) {
		if(!pkgConfigs.insert(std::pair<unsigned int, PackageConfiguration>(config.id, config)).second) {
			ROS_FATAL("Package configuration IDs are not unique!");
			return false;
		}
	}

	// build tray config map
	for(auto config : req.warehouse_configuration.trays) {
		if(!trayConfigs.insert(std::pair<unsigned int, Tray>(config.id, config)).second) {
			ROS_FATAL("Tray configuration IDs are not unique!");
			return false;
		}
	}

	// build robot config map
	for(auto config : req.robot_configurations) {
		if(!robotConfigs.insert(std::pair<std::string, RobotConfiguration>(config.type_name, config)).second) {
			ROS_FATAL("Robot configuration IDs are not unique!");
			return false;
		}
	}

	// subscribe to the storage update topic
	storageUpdateSub = n.subscribe("/storage_management/storage_update", 1000, &TaskPlanner::receiveStorageUpdate, this);
	robotHeartbeatSub = n.subscribe("/robot_heartbeats", 1000, &TaskPlanner::receiveRobotHeartbeat, this);

	newInputTaskServer = pn.advertiseService("new_input_task", &TaskPlanner::newInputRequest, this);
	newOutputTaskServer = pn.advertiseService("new_output_task", &TaskPlanner::newOutputRequest, this);
	registerAgentServer = pn.advertiseService("register_agent", &TaskPlanner::registerAgent, this);
	
	statusUpdatePub = pn.advertise<TaskPlannerState>("status", 1);
	rescheduleTimer = n.createTimer(ros::Duration(10.0), &TaskPlanner::rescheduleEvent, this);
	statusUpdateTimer = n.createTimer(ros::Duration(2.0), &TaskPlanner::taskStateUpdateEvent, this);
	taskResponseSub = n.subscribe("/task_response", 1000, &TaskPlanner::receiveTaskResponse, this);
	taskAnnouncerPub = pn.advertise<TaskAnnouncement>("task_broadcast", 1);

	ROS_INFO("Task planner initialized.");

	res.success = true;
	return true;
}

const PackageConfiguration& TaskPlanner::getPkgConfig(unsigned int typeId) const {
	return pkgConfigs.at(typeId);
}

const Tray& TaskPlanner::getTrayConfig(unsigned int trayId) const {
	return trayConfigs.at(trayId);
}

const RobotConfiguration& TaskPlanner::getRobotConfig(
		std::string robotId) const {
	return robotConfigs.at(robotId);
}

const std::map<unsigned int, Tray>& TaskPlanner::getTrayConfigs() const {
	return trayConfigs;
}

StorageState TaskPlanner::getStorageState() const {
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<GetStorageState>(
			"/storage_management/get_storage_information");
	GetStorageState srv;

	if(!client.call(srv)) {
		ROS_ERROR("Service call to get storage state failed!");
	}

	return srv.response.state;
}

void TaskPlanner::receiveStorageUpdate(const StorageUpdate& update) {
	// do the update only on relevant updates (end of reservation)
	if(update.action != StorageUpdate::DERESERVATION) {
		return;
	}

	resourceChangeEvent();
}

void TaskPlanner::receiveRobotHeartbeat(const auto_smart_factory::RobotHeartbeat& hb) {
	// get last robot status
	bool currentIdleStatus = registeredRobots[hb.id].second;

	// update idle status
	registeredRobots[hb.id].second = hb.idle;

	if(hb.idle != currentIdleStatus) {
		// robot status changed
		resourceChangeEvent();
	}
}

bool TaskPlanner::newInputRequest(auto_smart_factory::NewPackageInputRequest& req,
                                  auto_smart_factory::NewPackageInputResponse& res) {

	ROS_WARN("[Task Planner]: new InputRequest for packet %d", req.package.id);
	// create new input request
	PackageConfiguration pkgConfig = getPkgConfig(req.package.type_id);
	TaskRequirementsConstPtr taskRequirements = std::make_shared<const InputTaskRequirements>(pkgConfig,
	                                                                                          req.input_tray_id);
	RequestPtr inputRequest = std::make_shared<Request>(this, taskRequirements, "input");

	ROS_INFO("[request %d] New input request at input tray %d for package %d of type %d.", inputRequest->getId(), req.input_tray_id, req.package.id, req.package.type_id);
	
	inputRequests.push_back(inputRequest);
	try {
		// try to allocate resources for request
		TaskData taskData = inputRequest->allocateResources();

		// allocation was successful, create task
		TaskPtr inputTask = std::make_shared<Task>(inputRequest->getId(),
		                                           taskData);

		// remove the just pushed request
		inputRequests.pop_back();
		// start execution of task
		startTask(inputTask);
	} catch(std::runtime_error& e) {
		ROS_DEBUG(
				"[request %d] Resource allocation of new input request failed: %s",
				inputRequest->getId(), e.what());
	}

	res.success = true;
	return true;
}

bool TaskPlanner::newOutputRequest(NewPackageOutputRequest& req,
                                   NewPackageOutputResponse& res) {

	// check if there is already an output request at this tray
	for(const RequestPtr& r : outputRequests) {
		auto requirements = std::static_pointer_cast<const OutputTaskRequirements>(r->getRequirements());
		if(requirements->getKnownTrayId() == req.output_tray_id) {
			// output tray is already part of an output request
			res.success = false;
			return true;
		}
	}

	// create new output request
	PackageConfiguration pkgConfig = getPkgConfig(req.package.type_id);
	TaskRequirementsConstPtr taskRequirements = std::make_shared<const OutputTaskRequirements>(pkgConfig,
	                                                                                           req.output_tray_id);
	RequestPtr outputRequest = std::make_shared<Request>(this, taskRequirements, "output");

	ROS_INFO("[request %d] New output request at output tray %d for package type %d.", outputRequest->getId(), req.output_tray_id, req.package.type_id); outputRequests.push_back(outputRequest);
	try {
		// try to allocate resources for request
		TaskData taskData = outputRequest->allocateResources();

		// allocation was successful, create task
		TaskPtr outputTask = std::make_shared<Task>(outputRequest->getId(),
		                                            taskData);

		//remove the just added request
		outputRequests.pop_back();
		// start execution of task
		startTask(outputTask);
	} catch(std::runtime_error& e) {
		ROS_DEBUG(
				"[request %d] Resource allocation of new output request failed: %s",
				outputRequest->getId(), e.what());
	}

	res.success = true;
	return true;
}

const std::map<std::string, std::pair<RobotConfiguration, bool> >& TaskPlanner::getRegisteredRobots() const {
	return registeredRobots;
}

bool TaskPlanner::registerAgent(RegisterAgentRequest& req, RegisterAgentResponse& res) {
	if(registeredRobots.count(req.agent_id) == 0) {
		// add new registered robot
		registeredRobots[req.agent_id].first = req.robot_configuration;
		registeredRobots[req.agent_id].second = false;

		ROS_INFO("Registered agent: %s", req.agent_id.c_str());

		res.success = true;
	} else {
		res.success = false;
	}

	return true;
}

void TaskPlanner::rescheduleEvent(const ros::TimerEvent& e) {
	resourceChangeEvent();
}

void TaskPlanner::resourceChangeEvent() {
	return;
	
	ROS_INFO("[task planner] Resource change event!");

	if(!idleRobotAvailable()) {
		// there is no idle robot, so nothing can be started
		ROS_INFO("[task planner] No idle robot available. No check is performed.");
		return;
	}
	// first, check if any output request can be started
	std::vector<RequestPtr>::iterator outputRequest = outputRequests.begin();
	while(outputRequest != outputRequests.end()) {
		ROS_INFO("[task planner] Checking output request %d.", (*outputRequest)->getId());

		try {
			// try to allocate resources for request
			TaskData taskData = (*outputRequest)->allocateResources();

			// allocation was successful, create task
			TaskPtr outputTask = std::make_shared<Task>((*outputRequest)->getId(), taskData);

			// remove request from queue
			outputRequest = outputRequests.erase(outputRequest);

			// start execution of task
			startTask(outputTask);
		} catch(std::runtime_error& e) {
			ROS_DEBUG("[request %d] Resource allocation of output request failed: %s", (*outputRequest)->getId(), e.what());
			outputRequest++;
		}
	}

	// then, check if any input request can be started
	std::vector<RequestPtr>::iterator inputRequest = inputRequests.begin();
	while(inputRequest != inputRequests.end()) {
		ROS_INFO("[task planner] Checking input request %d.", (*inputRequest)->getId());

		// remove input request if it is not pending anymore
		if(!((*inputRequest)->isPending())) {
			ROS_INFO("[task planner] Input request %d is not pending anymore. It is deleted.", (*inputRequest)->getId());

			// remove request from queue
			inputRequest = inputRequests.erase(inputRequest);
			continue;
		}

		try {
			ROS_INFO("allocating input task data");

			// try to allocate resources for request
			TaskData taskData = (*inputRequest)->allocateResources();

			// allocation was successful, create task
			TaskPtr inputTask = std::make_shared<Task>((*inputRequest)->getId(), taskData);
			// remove request from queue
			inputRequest = inputRequests.erase(inputRequest);
			// start execution of task
			startTask(inputTask);

		} catch(std::runtime_error& e) {
			ROS_DEBUG("[request %d] Resource allocation of input request failed: %s", (*inputRequest)->getId(), e.what());
			inputRequest++;
		}
	}
}

void TaskPlanner::startTask(TaskPtr task) {
	// insert task into list of running tasks
	auto res = runningTasks.insert(std::pair<unsigned int, TaskPtr>(task->getId(), task));
	if(!res.second) {
		ROS_FATAL("Starting new task failed because task with same id is already running!");
		return;
	}

	// start task in new thread
	task->execute();
}

void TaskPlanner::taskStateUpdateEvent(const ros::TimerEvent& e) {
	auto_smart_factory::TaskPlannerState state;
	state.stamp = ros::Time::now();
	state.registered_robots = registeredRobots.size();

	// add states of requests
	for(auto& request : inputRequests) {
		state.requests.push_back(request->getStatus());
	}
	for(auto& request : outputRequests) {
		state.requests.push_back(request->getStatus());
	}

	for(auto taskIter = runningTasks.cbegin(); taskIter != runningTasks.cend();) {
		TaskPtr task = taskIter->second;

		// detect finished tasks
		if(task->getState().status == "finished") {
			if(!task->isJoinable()) {
				ROS_ERROR(
						"Task is marked as finished but thread is not joinable.");
			} else {
				// join taskIter thread
				task->join();
			}

			// remove from running list
			taskIter = runningTasks.erase(taskIter);
		} else {
			++taskIter;
		}

		// add state of taskIter
		state.tasks.push_back(task->getState());
	}

	// publish status
	statusUpdatePub.publish(state);
}

bool TaskPlanner::idleRobotAvailable() const {
	for(auto const& robot : registeredRobots) {
		if(robot.second.second) {
			return true;
		}
	}
	return false;
}

void TaskPlanner::receiveTaskResponse(const auto_smart_factory::TaskRating& tr){
	// go through requests and get the one for which the response is intended:
	for(RequestPtr& r : inputRequests){
		if(r->getId() == tr.request_id){
			r->receiveTaskResponse(tr);
			return;
		}
	}
	for(RequestPtr& r : outputRequests){
		if(r->getId() == tr.request_id){
			r->receiveTaskResponse(tr);
			return;
		}
	}
	ROS_WARN("No request with id %d found", tr.request_id);
}

void TaskPlanner::publishTask(const std::vector<auto_smart_factory::Tray>& sourceTrayCandidates,
                	 const std::vector<auto_smart_factory::Tray>& targetTrayCandidates, 
					 uint32_t requestId){
	TaskAnnouncement tsa;
	tsa.request_id = requestId;
	extractData(sourceTrayCandidates, targetTrayCandidates, &tsa);
	ROS_WARN("[Task Planner]: Publishing Request %d with %d start Trays and %d end Trays", tsa.request_id, (unsigned int)tsa.start_ids.size(), (unsigned int)tsa.end_ids.size());
	taskAnnouncerPub.publish(tsa);
}

void TaskPlanner::extractData(const std::vector<auto_smart_factory::Tray>& sourceTrays, const std::vector<auto_smart_factory::Tray>& targetTrays, auto_smart_factory::TaskAnnouncement* tsa){
	for(Tray t : sourceTrays){
		tsa->start_ids.push_back(t.id);
	}
	for(Tray t : targetTrays){
		tsa->end_ids.push_back(t.id);
	}
}