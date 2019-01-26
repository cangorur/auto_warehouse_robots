#ifndef AGENT_CHARGINGTASK_H_
#define AGENT_CHARGINGTASK_H_

#include "ros/ros.h"
#include "agent/task_handling/Task.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class ChargingTask : public Task
{
	public:

		explicit ChargingTask(uint32_t targetID, OrientedPoint targetPos, Path targetPath, double startTime);

		virtual ~ChargingTask();

		float getBatteryConsumption(void);

		float getDistance(void);

		double getDuration(void);

		void setState(Task::State state);

	protected:

		float chargingTime = 0;
};

#endif /* AGENT_CHARGINGTASK_H_ */