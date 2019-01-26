#ifndef AGENT_TRANSPORTATIONTASK_H_
#define AGENT_TRANSPORTATIONTASK_H_

#include "ros/ros.h"
#include "agent/Task.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class TransportationTask : public Task
{
	public:

		explicit TransportationTask(unsigned int id, uint32_t sourceID, OrientedPoint sourcePos, 
					uint32_t targetID, OrientedPoint targetPos, Path sourcePath, Path targetPath);

		virtual ~TransportationTask();

		unsigned int getId();

		uint32_t getSourceTrayId();

		OrientedPoint getSourcePosition();

		Path* getPathToSource();

		float getBatteryConsumption();

		float getDistance();

		void setState(Task::State state);

	protected:

		// the task id of the task
		unsigned int id;

		// source Tray id
		uint32_t sourceId;
		// source Tray position
		OrientedPoint sourcePosition;

		// Path to source
		Path pathToSource;

};

#endif /* AGENT_TRANSPORTATIONTASK_H_ */