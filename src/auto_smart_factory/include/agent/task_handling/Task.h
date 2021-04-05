#ifndef AGENT_TASK_H_
#define AGENT_TASK_H_

#include "ros/ros.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"
#include "auto_smart_factory/TaskEvaluation.h"
/**
 * The abstract Task class for the agent
 * Form this class the task subcategories inherit  
 * It contains the generic properties and functions of a task
 */
class Task {
public:
	/// Type for the different task types
	enum class Type {CHARGING, TRANSPORTATION};
	/// Type for the different states of a task
	enum class State {WAITING, TO_SOURCE, APPROACH_SOURCE, PICKUP, RESERVING_TARGET, TO_TARGET, APPROACH_TARGET, DROPOFF, LEAVE_TARGET, FINISHED, CHARGING};

	/**
	 * Constructor
	 * @param targetID, the id of the target tray
	 * @param targetPath, precomputed path to the target
	 * @param type, the type of the task
	 * @param startTime, the estimated time the task will start at 
	 */
	Task(uint32_t targetID, Path targetPath, Type type, double startTime);
	/**
	 * Destructor
	 */
	virtual ~Task() = default;
	
	/**
	 * getter for targetID
	 * @return uint32_t id
	 */
	uint32_t getTargetTrayId();

	/**
	 * getter for the target position
	 * @return OrientedPoint
	 */
	OrientedPoint getTargetPosition();

	/**
	 * returns the type of the task
	 * @return Task::Type
	 */
	Task::Type getType();

	/**
	 * Returns the state the task is in
	 * @return Task::State
	 */
	Task::State getState();

	/**
	 * Wrapper returning if a task is a transportation task
	 * @return bool
	 */
	bool isTransportation();
	/**
	 * Wrapper returning if a task is a charging task
	 * @return bool
	 */
	bool isCharging();

	/**
	 * Set state of the task
	 * @param state, to be set
	 */
	virtual void setState(Task::State state) = 0;

	/**
	 * Return the estimated battery consumption of this task
	 * @return estimated battery consumption
	 */
	virtual double getBatteryConsumption() = 0;

	/**
	 * Return the estimated duration
	 * @return estimated duration
	 */
	virtual double getDuration() = 0;

	/**
	 * Function filling the given TaskEvaluation message with the timestamps
	 * @param msg, message to be filled
	 */
	virtual void fillInEvaluationData(auto_smart_factory::TaskEvaluation* msg) = 0;

	/**
	 * returns the estimated end time of this task
	 * @return estimated end time
	 */
	double getEndTime();

	/**
	 * Sets the battery level at the beginning of a task.
	 * Will only change something if the task is in the WAITING state
	 * sets the batteryStart variable
	 * @param batteryLevel the current charge of the battery
	 */
	void setStartBatteryLevel(double batteryLevel);	

	/**
	 * Sets the battery level at the end of a task.
	 * Will only change something if the task is in the FINISHED state for a transportation task
	 * Will only change something if the task is in the TO_TARGET state for a charging task (do this continuously while in that state)
	 * sets the batteryFinish variable
	 * @param batteryLevel the current charge of the battery
	 */
	void setFinishBatteryLevel(double batteryLevel);

protected:
	// type of the task	
	Task::Type type;
	// state of the task
	Task::State state;

	// the estimated moment in time when the task will start execution
	double startTime;
	// the estimated time the task will take
	double targetDuration;
	// the estimated battery consumption the task will consume
	double targetBatCons;

	// target Tray id
	uint32_t targetId;
	// target Tray position
	OrientedPoint targetPosition;

	// Time information for post-mortem analysis and evaluation
	double assignedAt;
	double startedAt;
	double finishedAt;
	// battery information for post-mortem analysis and evaluation
	double batteryStart;
	double batteryFinish;
};

#endif /* AGENT_TASK_H_ */