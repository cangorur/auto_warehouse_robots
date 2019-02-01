#ifndef AGENT_TASKHANDLER_H_
#define AGENT_TASKHANDLER_H_

#include <list>
#include <string>

#include "ros/ros.h"
#include "agent/task_handling/Task.h"
#include "agent/task_handling/TransportationTask.h"
#include "agent/task_handling/ChargingTask.h"
#include "agent/path_planning/ReservationManager.h"
#include "auto_smart_factory/TaskAnnouncement.h"
#include "auto_smart_factory/TaskRating.h"
#include "agent/path_planning/Map.h"
#include "agent/MotionPlanner.h"
#include "agent/Gripper.h"
#include "agent/ChargingManagement.h"

class TaskHandler
{
	public:
    	explicit TaskHandler(std::string agentId, ros::Publisher* scorePublish, Map* map, MotionPlanner* mp, Gripper* gripper, ChargingManagement* cm, ReservationManager* rm);

    	void publishScore(unsigned int requestId, double score, uint32_t startTrayId, uint32_t endTrayId);
		void rejectTask(unsigned int requestId);

		void update(void);

    	virtual ~TaskHandler();

    	void addTransportationTask(unsigned int id, uint32_t sourceID, uint32_t targetID, 
				Path sourcePath, Path targetPath, double startTime);

    	void addChargingTask(uint32_t targetID, Path targetPath, double startTime);

		void executeTask(void);

    	void nextTask(void);

		bool isTaskInExecution(void);

		bool isIdle(void);

		unsigned int numberQueuedTasks(void);

		Task* getCurrentTask(void);

		double getBatteryConsumption(void);

		double getDuration(void);

		Task* getLastTask(void);

	private:
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

		// the id of the agent to which this taskHandler belongs
	    std::string agentId;

		// a pointer to the publisher for the score
    	ros::Publisher* scorePublisher;
};

#endif /* AGENT_TASKHANDLER_H_ */