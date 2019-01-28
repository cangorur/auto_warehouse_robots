#ifndef AGENT_TASK_H_
#define AGENT_TASK_H_

#include "ros/ros.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class Task
{
  public:
		enum class Type {CHARGING, TRANSPORTATION};
		enum class State {WAITING, TO_SOURCE, PICKUP, RESERVING_TARGET, TO_TARGET, DROPOFF, FINISHED, CHARGING};

		Task(uint32_t targetID, Path targetPath, Type type, double startTime);

		uint32_t getTargetTrayId(void);

		OrientedPoint getTargetPosition(void);

		Task::Type getType(void);

		Task::State getState(void);

		bool isTransportation(void);
		bool isCharging(void);

		virtual void setState(Task::State state) = 0;

		virtual double getBatteryConsumption(void) = 0;

		virtual double getDuration(void) = 0;

		// returns the estimated end time of this task
		double getEndTime(void);

	protected:	
		Task::Type type;
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
};

#endif /* AGENT_TASK_H_ */