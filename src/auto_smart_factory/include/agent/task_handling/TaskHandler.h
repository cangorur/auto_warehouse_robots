#ifndef AGENT_TASKHANDLER_H_
#define AGENT_TASKHANDLER_H_

#include <list>
#include <string>

#include "ros/ros.h"
#include "agent/Agent.h"
#include "agent/task_handling/Task.h"
#include "agent/task_handling/TransportationTask.h"
#include "agent/task_handling/ChargingTask.h"
#include "agent/task_handling/TrayScore.h"
#include "agent/path_planning/ReservationManager.h"
#include "auto_smart_factory/TaskAnnouncement.h"
#include "auto_smart_factory/TaskRating.h"
#include "auto_smart_factory/TaskEvaluation.h"
#include "auto_smart_factory/TaskStarted.h"
#include "agent/path_planning/Map.h"
#include "agent/MotionPlanner.h"
#include "agent/Gripper.h"
#include "agent/ChargingManagement.h"

class Agent;

class TaskHandler
{
	public:
		/**
		 * Constructor
		 * @param agent, a pointer to the agent constructing the task handler
		 * @param scorePublish, a pointer to the publisher for TaskScore messages
		 * @param evalPub, a pointer to the publisher for TaskEvaluation messages
		 * @param startedPub, a pointer to the publisher for TaskStarted messages
		 * @param map, a pointer to the map of the agent
		 * @param mp, a pointer to the motion planner of the agent
		 * @param gripper, a pointer to the gripper of the agent
		 * @param cm, a pointer to the charging management of the agent
		 * @param rm, a pointer to the reservation manager of the agent
		 * @return TaskHandler object
		 */
    	explicit TaskHandler(Agent* agent, ros::Publisher* scorePublish, ros::Publisher* evalPub, ros::Publisher* startedPub, Map* map, MotionPlanner* mp, Gripper* gripper, ChargingManagement* cm, ReservationManager* rm);

		/**
		 * destructor
		 */
    	virtual ~TaskHandler();

		/**
		 * Manages the current state of the TaskHandler, executes tasks if one is assigned, answers taskAnnouncements
		 * Should be called in an spin-one loop(see Agent.cpp)
		 */
		void update();

		/**
		 * Returns if a task is in execution
		 * @return bool 
		 */
		bool isTaskInExecution();

		/**
		 * Returns if the robot is idle
		 * @retun bool
		 */
		bool isIdle();

		/**
		 * Return the number of queued Tasks
		 * @return unsigned int number of queued tasks
		 */
		unsigned int numberQueuedTasks();

		/**
		 * Returns a pointer to the task being executed currently
		 * Returns nullptr if there is no current task
		 * @return a pointer to a task bject or nullptr
		 */
		Task* getCurrentTask();

		/**
		 * Returns the estimatedd battery level after all the queued tasks
		 * @return double
		 */
		double getEstimatedBatteryLevelAfterQueuedTasks();

		/**
		 * Returns the estimated duration for all queued tasks
		 * @return double
		 */
		double getDuration();

		/**
		 * Returns a pointer to the last task in queue, or the current task if the queue is empty. 
		 * may be a nullptr if no task is queued and the robot is idle
		 * @return pointer to Task object or nullptr
		 */
		Task* getLastTask();

		/**
		 * The callback function to receive a TaskAnnouncement message
		 */
		void announcementCallback(const auto_smart_factory::TaskAnnouncement& tA);

		/**
		 * Add a transportation Task to the queue
		 * @param id, the id of the task to be added
		 * @param sourceID, the id of the source tray
		 * @param targetID, the id of the target tray
		 * @param sourcePath, precomputed path to the source tray
		 * @param taretPath, precomputed path to the target tray
		 * @param startTime, time at which the task is estimated to start
		 */
    	void addTransportationTask(unsigned int id, uint32_t sourceID, uint32_t targetID, Path sourcePath, Path targetPath, double startTime);

		/**
		 * Add a charging task to the queue
		 * @param targetID, the id of the charging station
		 * @param targetPath, precomputed path to the charging station
		 * @param startTime, time at which the task is estimated to start
		 */
    	void addChargingTask(uint32_t targetID, Path targetPath, double startTime);
	
	private:

		/**
		 * Advancing and Executing a task
		 * is called in the TaskHandler update function
		 */
		void executeTask();

		/**
		 * Function starting the nexxt task from the queue
		 * is called in the TaskHandler update function
		 */
    	void nextTask();

		/**
		 * function for publishing a score as a response to a task announcement,
		 * this response will be a positive one, meaning the robot can accept the task
		 * @param requestId, the id of the task which was announced
		 * @param score, the computed score
		 * @param startTrayId, the id of the source tray from which the robot intends to pick up the package
		 * @param endTrayId, the id of the target tray at which the robot intends to drop off the package
		 * @param estimatedDuration, an estimation of how long the robot will take to finish the task
		 */
    	void publishScore(unsigned int requestId, double score, uint32_t startTrayId, uint32_t endTrayId, double estimatedDuration);

		/**
		 * function for publishing a denial as a response to a task announcement,
		 * this response will be a negative one, meaning the robot can not accept the task
		 * @param requestId, the id of the task which was announced
		 */
		void rejectTask(unsigned int requestId);

		/**
		 * function for replanning when something goes wrong
		 */
		void replan();

		/**
		 *  function for sending evaluation data of the current task to the Evaluator node
		 */
		void sendEvaluationData();

		/**
		 * Answer all announcements in the announcement queue
		 */
		void answerAnnouncements();

		/**
		 * Answer the given announcement
		 * @param taskannouncement
		 */
		void answerAnnouncement(auto_smart_factory::TaskAnnouncement& taskAnnouncement);

		/**
		 * TODO: VINCENT
		 */
		double getApproachDistance(OrientedPoint robotPos, OrientedPoint pathTargetPos) const;
		
		// the current task
    	Task* currentTask = nullptr;

		// the list of planned Tasks
    	std::list<Task*> queue;

		// the map to calculate the path(s) for the score
		Map* map;

		// references to MotionPlanner, Gripper, Charging Management and Reservation Manager
		MotionPlanner* motionPlanner;
		Gripper* gripper;
		ChargingManagement* chargingManagement;
		ReservationManager* reservationManager;

		// a pointer to the agent to which this taskHandler belongs
	    Agent* agent;

		// a pointer to the publisher for the score
    	ros::Publisher* scorePublisher;

		// a pointer to the publisher for evaluations
		ros::Publisher* evalPub;

		// a pointer to the publisher for started tasks
		ros::Publisher* startedPub;

		// the task was changed to the next one in queue
		bool isNextTask = false;

		// the current task has tried to reserve a path to target
		bool hasTriedToReservePathToTarget = false;

		// is currently replanning
		bool isReplanning = false;

		// the list of unanswered rask announcements
		std::list<auto_smart_factory::TaskAnnouncement> announcements;
		
		// TODO: VINCENT
		double lastApproachDistance;
};

#endif /* AGENT_TASKHANDLER_H_ */