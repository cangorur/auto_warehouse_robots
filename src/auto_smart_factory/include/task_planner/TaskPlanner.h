/*
 * TaskPlanner.h
 *
 *  Created on: 09.07.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASKPLANNER_H_
#define AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASKPLANNER_H_

#include "ros/ros.h"
#include <memory>
#include <thread>

#include "auto_smart_factory/InitTaskPlanner.h"
#include "auto_smart_factory/PackageConfiguration.h"
#include "auto_smart_factory/Tray.h"
#include "auto_smart_factory/RobotConfiguration.h"
#include "auto_smart_factory/RobotHeartbeat.h"
#include "auto_smart_factory/StorageUpdate.h"
#include "auto_smart_factory/GetStorageState.h"
#include "auto_smart_factory/NewPackageInput.h"
#include "auto_smart_factory/NewPackageOutput.h"
#include "auto_smart_factory/RegisterAgent.h"
#include "auto_smart_factory/TaskPlannerState.h"

#include "task_planner/Task.h"
#include "auto_smart_factory/Tray.h"
#include "task_planner/Request.h"

/**
 * The task planner component manages all incoming requests, checks for resources
 * and starts new tasks.
 */
class TaskPlanner {
public:
	TaskPlanner();
	virtual ~TaskPlanner() = default;

	/**
	 * Get package configuration of a given package type id.
	 * @param typeId Package type id
	 * @return Package configuration
	 */
	const auto_smart_factory::PackageConfiguration& getPkgConfig(unsigned int typeId) const;

	/**
	 * Get tray configuration of a given tray id.
	 * @param trayId Tray id
	 * @return Tray configuration
	 */
	const auto_smart_factory::Tray& getTrayConfig(unsigned int trayId) const;

	/**
	 * Get robot configuration of a given robot id.
	 * @param robotId Robot id
	 * @return Robot configuration
	 */
	const auto_smart_factory::RobotConfiguration& getRobotConfig(std::string robotId) const;

	/**
	 * Get all tray configurations.
	 * @return Tray configurations
	 */
	const std::map<unsigned int, auto_smart_factory::Tray>& getTrayConfigs() const;

	/**
	 * Get list of all registered robots with their idle status.
	 * @return Map of registered robots <robot id, (robot configuration, idle status)>
	 */
	const std::map<std::string, std::pair<auto_smart_factory::RobotConfiguration, bool> >& getRegisteredRobots() const;


	void publishTask(const std::vector<auto_smart_factory::Tray>& sourceTrayCandidates,
                	 const std::vector<auto_smart_factory::Tray>& targetTrayCandidates, 
					 uint32_t requestId);

	/* 
	 * Function checking if something has changed and if so tries to assign not yet assigned tasks
	 */
	void update();

private:
	/**
	 * Initialize service handler.
	 * @param req Request object
	 * @param res Response object
	 * @return True if initialization was successful
	 */
	bool initialize(auto_smart_factory::InitTaskPlannerRequest& req, auto_smart_factory::InitTaskPlannerResponse& res);

	/**
	 * Receive storage updates.
	 * @param update Storage update
	 */
	void receiveStorageUpdate(const auto_smart_factory::StorageUpdate& update);

	/**
	 * Receive robot heartbeats.
	 * @param hb Heartbeast message
	 */
	void receiveRobotHeartbeat(const auto_smart_factory::RobotHeartbeat& hb);

	/**
	 * Handle new input request.
	 *
	 * \note It is assumed that the package was already put into the specified input tray.
	 *
	 * @param req Request object with tray and package
	 * @param res Response object
	 * @return Always true
	 */
	bool newInputRequest(auto_smart_factory::NewPackageInputRequest& req, auto_smart_factory::NewPackageInputResponse& res);

	/**
	 * Handle new output request.
	 * @param req Request object with output tray and package type
	 * @param res Response object
	 * @return Always true
	 */
	bool newOutputRequest(auto_smart_factory::NewPackageOutputRequest& req,
	                      auto_smart_factory::NewPackageOutputResponse& res);

	/**
	 * Called by robots. Registers a robot to the task planner.
	 * @param req Request object
	 * @param res Response object
	 * @return Always true
	 */
	bool registerAgent(auto_smart_factory::RegisterAgentRequest& req, auto_smart_factory::RegisterAgentResponse& res);

	/**
	 * Get current storage state from the storage management.
	 * @return Storage state
	 */
	auto_smart_factory::StorageState getStorageState() const;

	/**
	 * Every 10 seconds this event triggers checking of all requests.
	 * @param e
	 */
	void rescheduleEvent(const ros::TimerEvent& e);

	/**
	 * This method is called whenever new resources get available.
	 * It checks all pending requests (output requests first) and
	 * tries to start new tasks.
	 */
	void resourceChangeEvent();

	/**
	 * This method is called regularly and publishes the current task planner
	 * status containing request and task states.
	 * @param e
	 */
	void taskStateUpdateEvent(const ros::TimerEvent& e);

	/**
	 * Starts a created task.
	 * @param task New task
	 */
	void startTask(TaskPtr task);

	/**
	 * Checks if at least one idle robot is available.
	 * @return True if an idle robot is available
	 */
	bool idleRobotAvailable() const;

	void receiveTaskResponse(const auto_smart_factory::TaskRating& tr);

	/// extract tray data into points and ids
	void extractData(const std::vector<auto_smart_factory::Tray>& sourceTrays, const std::vector<auto_smart_factory::Tray>& targetTrays, auto_smart_factory::TaskAnnouncement* tsa);


private:
	/// flag indicating that something has changed and that the task planner should try to assign tasks
	bool resourcesChanged = false;

	// frequency with which the task planner checks if it can assign previously unassigned tasks
	ros::Duration updateFrequency;

	/// the thread in which the task planner update function is working
	std::thread updateExecutionThread;

	/// all registered robots: mapping their id to their configuration and their state (idle/busy with idle = true)
	std::map<std::string, std::pair<auto_smart_factory::RobotConfiguration, bool> > registeredRobots;

	/// List of pending input requests
	std::vector<Request> inputRequests;

	/// List of pending ouput requests
	std::vector<Request> outputRequests;

	/// Map of running tasks
	std::map<unsigned int, TaskPtr> runningTasks;

	/// All package configurations
	std::map<unsigned int, auto_smart_factory::PackageConfiguration> pkgConfigs;

	/// All tray configurations
	std::map<unsigned int, auto_smart_factory::Tray> trayConfigs;

	/// all robot configurations
	std::map<std::string, auto_smart_factory::RobotConfiguration> robotConfigs;

	/// timer used to regularly reschedule
	ros::Timer rescheduleTimer;

	/// Task planner status publisher
	ros::Publisher statusUpdatePub; 

	/// Task planner status update timer
	ros::Timer statusUpdateTimer;

	/// subscriber to the storage update topic
	ros::Subscriber storageUpdateSub;

	/// Subscriber to robot heartbeat topic
	ros::Subscriber robotHeartbeatSub;

	/// Server for new input request
	ros::ServiceServer newInputTaskServer;

	/// Server for new output request
	ros::ServiceServer newOutputTaskServer;

	/// Server for agent/robot registration
	ros::ServiceServer registerAgentServer;

	/// Service for initialization
	ros::ServiceServer initServer;

	/// Subscriber to robot task response topic
	ros::Subscriber taskResponseSub;

	/// Task planner task announcement publisher
	ros::Publisher taskAnnouncerPub;
};

#endif /* AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASKPLANNER_H_ */
