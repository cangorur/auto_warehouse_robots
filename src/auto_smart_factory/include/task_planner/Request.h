/*
 * Request.h
 *
 *  Created on: 21.07.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_TASK_PLANNER_REQUEST_H_
#define AUTO_SMART_FACTORY_SRC_TASK_PLANNER_REQUEST_H_

#include "ros/ros.h"

#include "auto_smart_factory/Tray.h"
#include "auto_smart_factory/RequestStatus.h"

#include "task_planner/InputTaskRequirements.h"
#include "task_planner/OutputTaskRequirements.h"
#include "task_planner/TaskData.h"

#include "auto_smart_factory/TaskAnnouncement.h"
#include "auto_smart_factory/TaskRating.h"

class TaskPlanner;

/**
 * This class represents a generic request. The properties of this request are
 * defined by the task requirement object specified on construction.
 */
class Request {
public:
	/**
	 * Create a request.
	 * @param tp Pointer to the task planner object
	 * @param taskRequirements The task requirements object defining the properties of this request
	 * @param type String representation of the task type, e.g. 'input' or 'output'
	 */
	Request(TaskPlanner* tp, TaskRequirementsConstPtr taskRequirements, std::string type);

	virtual ~Request() = default;

	/**
	 * Tries to allocate all necessary resources to start a task.
	 *
	 * \throws std::runtime_error with failure description if allocation is not successful
	 *
	 * @return TaskData object that contains all necessary information about allocated resources needed to start a task
	 */
	TaskData allocateResources();

	/**
	 * Check if this request is still pending or can be deleted.
	 * @return True if it is still pending.
	 */
	bool isPending() const;

	/**
	 * Returns id of this request
	 * @return Request id
	 */
	unsigned int getId() const;

	/**
	 * Return status information about this request.
	 * @return Status information
	 */
	auto_smart_factory::RequestStatus getStatus() const;

	/**
	 * Get request's requirements.
	 * @return Requirements object
	 */
	TaskRequirementsConstPtr getRequirements() const;

	/*
	 * Receive response to an task announcement
	 * @param Answer of a robot as TaskRating message
	 */
	void receiveTaskResponse(const auto_smart_factory::TaskRating& tr);

	/*
	 * Returns if the Request is currently waiting for answers
	 * @return busy information
	 */
	bool isBusy();

	/// the time the robots have to answer the request
	static const ros::Duration timeoutDuration;

protected:
	/**
	 * Creates a list of possible source tray candidates for this request.
	 * @param sourceTrayCandidates Output vector for candidates
	 * @return True if candidate list is non-empty
	 */
	bool findSourceCandidates(std::vector<auto_smart_factory::Tray>& sourceTrayCandidates) const;

	/**
	 * Creates a list of possible target tray candidates for this request.
	 * @param targetTrayCandidates Output vector for candidates
	 * @return True if candidate list is non-empty
	 */
	bool findTargetCandidates(std::vector<auto_smart_factory::Tray>& targetTrayCandidates) const;

	/**
	 * Creates a list of possible robot candidates using lists of source and target tray candidates.
	 * All robots are queried using the candidates.
	 * This function will wait for either a timeout or until all robots have answered the announcement made in this function.
	 * NOTE: While the function is waiting it calls the spinOnce() function
	 * @param robotCandidates Output vector for robot candidates
	 * @param sourceTrayCandidates List of source tray candidates
	 * @param targetTrayCandidates List of output tray candidates
	 * @return True if robot candidate list is non-empty
	 */
	bool getRobotCandidates(const std::vector<auto_smart_factory::Tray>& sourceTrayCandidates, const std::vector<auto_smart_factory::Tray>& targetTrayCandidates);

	/**
	 * Assign request/task to robot for includes the setup of the task in the robot.
	 *
	 * @param robotId Id of the robot
	 * @return True if assigning was successful
	 */
	bool allocateRobot(const RobotCandidate& candidate) const;

protected:
	/// the request status
	auto_smart_factory::RequestStatus status;

	/// reference to the task planner
	TaskPlanner* taskPlanner;

	/// requirements that need to be fulfilled
	TaskRequirementsConstPtr requirements;

	/// vector of robot candidates
	std::vector<RobotCandidate> robotCandidates;

	/// a map of robots who answered with robot_id as key and their reject flag as value
	std::map<std::string, bool> answeredRobots;

	/// variable guarding that no scores are accepted after the timeout
	bool acceptingScores;

protected:
	/// used to generate unique ids
	static unsigned int nextId;

	/// generate new unique id
	static unsigned int getNewId();

	/**
	 * wait with a frequency until each robot has answered or a timeout occurs
	 * @param timeout, maximum waiting time
	 * @param frequency, the frequency with which the function will check if all answers were received
	 */
	void waitForRobotScores(ros::Duration timeout, ros::Rate frequency);
};

typedef std::shared_ptr<Request> RequestPtr;

#endif /* AUTO_SMART_FACTORY_SRC_TASK_PLANNER_REQUEST_H_ */
