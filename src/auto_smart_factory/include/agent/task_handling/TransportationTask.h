#ifndef AGENT_TRANSPORTATIONTASK_H_
#define AGENT_TRANSPORTATIONTASK_H_

#include "ros/ros.h"
#include "agent/task_handling/Task.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

/**
 * The transportation task represents a subcategory of a task for 
 * which robots have to drive to a source location and from there to a target location
 */
class TransportationTask : public Task {
public:
	/**
	 * Constructor for a Transportation Task
	 * @param id, the id of the task to be added
	 * @param sourceID, the id of the source tray
	 * @param targetID, the id of the target tray
	 * @param sourcePath, precomputed path to the source tray
	 * @param taretPath, precomputed path to the target tray
	 * @param startTime, time at which the task is estimated to start
	 */
	explicit TransportationTask(unsigned int id, uint32_t sourceID, uint32_t targetID, Path sourcePath, Path targetPath, double startTime);
	~TransportationTask() override = default;

	/**
	 * Return the id of the transportation task
	 * @return id
	 */
	unsigned int getId();

	/**
	 * getter for sourceID
	 * @return uint32_t id
	 */
	uint32_t getSourceTrayId();
	
	/**
	 * getter for the source position
	 * @return OrientedPoint
	 */
	OrientedPoint getSourcePosition();
	
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
	 * Return the estimation of time needed to pick up a packet
	 */
	static double getPickUpTime();

	/**
	 * Return the estimation of time needed to drop off a packet
	 */
	static double getDropOffTime();
	
protected:
	// the task id of the task
	unsigned int id;

	// source Tray id
	uint32_t sourceId;
	// source Tray position
	OrientedPoint sourcePosition;
	
	double sourceDuration;
	double sourceBatCons;

	static double pickUpTime;
	static double dropOffTime;

	// Time information for post-mortem analysis
	double startedPickUpAt;
	double finishedPickUpAt;
	double startedDropOffAt;
};

#endif /* AGENT_TRANSPORTATIONTASK_H_ */