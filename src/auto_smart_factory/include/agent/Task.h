#ifndef AGENT_TASK_H_
#define AGENT_TASK_H_

#include "ros/ros.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class Task
{
  public:

		enum class taskType {CHARGING, TRANSPORTATION};

		Task(uint32_t targetID, OrientedPoint targetPos, Path targetPath, taskType type);

		uint32_t getTargetTrayId(void);

		OrientedPoint getTargetPosition(void);

		Path* getPathToTarget(void);

		Task::taskType getType(void);

		// TODO: add estimated battery consumption function
		virtual float getBatteryConsumption(void) = 0;

		virtual float getDistance(void) = 0;

	protected:

		Task::taskType type;

		// target Tray id
		uint32_t targetId;
		// target Tray position
		OrientedPoint targetPosition;

		// calculated path from source to target (may be Theta* or RRT*)
		Path pathToTarget;

};

#endif /* AGENT_TASK_H_ */