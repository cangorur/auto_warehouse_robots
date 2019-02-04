#ifndef AGENT_CHARGINGTASK_H_
#define AGENT_CHARGINGTASK_H_

#include "ros/ros.h"
#include "agent/task_handling/Task.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class ChargingTask : public Task {
public:
	explicit ChargingTask(uint32_t targetID, Path targetPath, double startTime);
	~ChargingTask() override = default;
	
	double getBatteryConsumption() override;
	double getDuration() override;
	void setState(Task::State state) override;
	
	static double getChargingTime();

protected:
	static double chargingTime;
};

#endif /* AGENT_CHARGINGTASK_H_ */