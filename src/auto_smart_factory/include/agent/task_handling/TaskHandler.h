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
#include "agent/path_planning/Map.h"
#include "agent/MotionPlanner.h"
#include "agent/Gripper.h"
#include "agent/ChargingManagement.h"

class Agent;

class TaskHandler
{
	public:
    	explicit TaskHandler(Agent* agent, ros::Publisher* scorePublish, Map* map, MotionPlanner* mp, Gripper* gripper, ChargingManagement* cm, ReservationManager* rm);

    	void publishScore(unsigned int requestId, double score, uint32_t startTrayId, uint32_t endTrayId, double estimatedDuration);
		void rejectTask(unsigned int requestId);

		void update();

    	virtual ~TaskHandler();

    	void addTransportationTask(unsigned int id, uint32_t sourceID, uint32_t targetID, Path sourcePath, Path targetPath, double startTime);

    	void addChargingTask(uint32_t targetID, Path targetPath, double startTime);

		void executeTask();

    	void nextTask();

		bool isTaskInExecution();

		bool isIdle();

		unsigned int numberQueuedTasks();

		Task* getCurrentTask();

		double getEstimatedBatteryLevelAfterQueuedTasks();

		double getDuration();

		Task* getLastTask();

		void announcementCallback(const auto_smart_factory::TaskAnnouncement& tA);

	private:
		void answerAnnouncements();

		void answerAnnouncement(auto_smart_factory::TaskAnnouncement& taskAnnouncement);
		
		// the current task
    	Task* currentTask = nullptr;

		// the list of planned Tasks
    	std::list<Task*> queue;

		// the map to calculate the path(s) for the score
		Map* map;

		// references to MotionPlanner, Gripper and Charging Management
		MotionPlanner* motionPlanner;
		Gripper* gripper;
		ChargingManagement* chargingManagement;
		ReservationManager* reservationManager;

		// a pointer to the agent to which this taskHandler belongs
	    Agent* agent;

		// a pointer to the publisher for the score
    	ros::Publisher* scorePublisher;

		// the task was changed to the next one in queue
		bool isNextTask = false;

		// the current task has tried to reserve a path to target
		bool hasTriedToReservePathToTarget = false;

		// the list of unanswered rask announcements
		std::list<auto_smart_factory::TaskAnnouncement> announcements;
};

#endif /* AGENT_TASKHANDLER_H_ */