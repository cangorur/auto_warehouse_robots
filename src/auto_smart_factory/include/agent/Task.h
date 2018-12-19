#ifndef AGENT_TASK_H_
#define AGENT_TASK_H_

#include "ros/ros.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/Path.h"

class Task
{
  public:
	explicit Task(unsigned int id, uint32_t sourceID, OrientedPoint sourcePos, 
				uint32_t targetID, OrientedPoint targetPos, Path sourcePath, Path targetPath);

	virtual ~Task();

	unsigned int getId();

	uint32_t getSourceTrayId();

	OrientedPoint getSourcePosition();

	uint32_t getTargetTrayId();

	OrientedPoint getTargetPosition();

	Path* getPathToTarget();

	Path* getPathToSource();

  private:
	// the task id of the task
	unsigned int id;

	// source Tray id
	uint32_t sourceId;
	// source Tray position
	OrientedPoint sourcePosition;

	// target Tray id
	uint32_t targetId;
	// target Tray position
	OrientedPoint targetPosition;

	// calculated path from source to target (may be Theta* or RRT*)
	Path pathToTarget;

	// Path to source
	Path pathToSource;

};

#endif /* AGENT_TASK_H_ */