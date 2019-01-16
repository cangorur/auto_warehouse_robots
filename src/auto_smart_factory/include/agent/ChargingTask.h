#ifndef AGENT_CHARGINGTASK_H_
#define AGENT_CHARGINGTASK_H_

#include "ros/ros.h"
#include "agent/Task.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class ChargingTask : public Task
{
	public:

		explicit ChargingTask(uint32_t targetID, OrientedPoint targetPos, Path targetPath);

		virtual ~ChargingTask();

		float getBatteryConsumption(void);

		float getDistance(void);

	protected:

};

#endif /* AGENT_CHARGINGTASK_H_ */