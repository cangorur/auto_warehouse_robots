#ifndef AGENT_TRANSPORTATIONTASK_H_
#define AGENT_TRANSPORTATIONTASK_H_

#include "ros/ros.h"
#include "agent/task_handling/Task.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class TransportationTask : public Task {
public:
	explicit TransportationTask(unsigned int id, uint32_t sourceID, uint32_t targetID, Path sourcePath, Path targetPath, double startTime);
	~TransportationTask() override = default;

	unsigned int getId();
	uint32_t getSourceTrayId();
	OrientedPoint getSourcePosition();
	double getBatteryConsumption() override;
	double getDuration() override;
	void setState(Task::State state) override;
	
	static double getPickUpTime();
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
};

#endif /* AGENT_TRANSPORTATIONTASK_H_ */