#ifndef AGENT_CHARGINGTASK_H_
#define AGENT_CHARGINGTASK_H_

#include "ros/ros.h"
#include "agent/task_handling/Task.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class ChargingTask : public Task {
public:
	/**
	 * Constructor
	 * @param targetID, the id of the target tray
	 * @param targetPath, precomputed path to the target
	 * @param type, the type of the task
	 * @param startTime, the estimated time the task will start at 
	 */
	explicit ChargingTask(uint32_t targetID, Path targetPath, double startTime);
	/**
	 * Destructor
	 */
	~ChargingTask() override = default;
	
	/**
	 * Return the estimated battery consumption of this task
	 * @return estimated battery consumption
	 */
	double getBatteryConsumption() override;
	
	/**
	 * Return the estimated duration
	 * @return estimated duration
	 */
	double getDuration() override;
	
	/**
	 * Set state of the task
	 * @param state, to be set
	 */
	void setState(Task::State state) override;

	/**
	 * Function filling the given TaskEvaluation message with the timestamps
	 * @param msg, message to be filled
	 */
	void fillInEvaluationData(auto_smart_factory::TaskEvaluation* msg) override;
	
	/**
	 * Return the estimation of time needed to charge
	 */
	static double getChargingTime();

	/**
	 * update the charging task with new target and startTime
	 */
	void adjustChargingStation(uint32_t targetID, Path targetPath, double startTime);

protected:
	// estimation for time needed to charge
	static double chargingTime;

	// Time information for post-mortem analysis
	double arrivedAt;
};

#endif /* AGENT_CHARGINGTASK_H_ */