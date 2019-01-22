#ifndef AGENT_TASK_H_
#define AGENT_TASK_H_

#include "ros/ros.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class Task
{
  public:
		enum class Type {CHARGING, TRANSPORTATION};
		enum class State {WAITING, TO_SOURCE, PICKUP, TO_TARGET, DROPOFF, FINISHED, CHARGING};

		Task(uint32_t targetID, OrientedPoint targetPos, Path targetPath, Type type);

		uint32_t getTargetTrayId(void);

		OrientedPoint getTargetPosition(void);

		Path* getPathToTarget(void);

		Task::Type getType(void);

		Task::State getState(void);

		bool isTransportation(void);
		bool isCharging(void);

		virtual void setState(Task::State state) = 0;

		virtual float getBatteryConsumption(void) = 0;

		virtual float getDistance(void) = 0;

	protected:	
		Task::Type type;
		Task::State state;

		// target Tray id
		uint32_t targetId;
		// target Tray position
		OrientedPoint targetPosition;

		// calculated path from source to target (may be Theta* or RRT*)
		Path pathToTarget;

};

#endif /* AGENT_TASK_H_ */