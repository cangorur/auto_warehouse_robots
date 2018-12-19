#include "agent/Task.h"

Task::Task(unsigned int id, uint32_t sourceID, OrientedPoint sourcePos, 
			uint32_t targetID, OrientedPoint targetPos, Path sourcePath, Path targetPath) : 
	id(id), 
	sourceId(sourceID),
	sourcePosition(sourcePos),
	targetId(targetID),
	targetPosition(targetPos),
	pathToSource(sourcePath),
	pathToTarget(targetPath) {
}

Task::~Task() = default;

unsigned int Task::getId(){
	return id;
}

uint32_t Task::getSourceTrayId(){
	return sourceId;
}

OrientedPoint Task::getSourcePosition(){
	return sourcePosition;
}

uint32_t Task::getTargetTrayId(){
	return targetId;
}

OrientedPoint Task::getTargetPosition(){
	return targetPosition;
}

Path* Task::getPathToTarget(){
	return &pathToTarget;
}

Path* Task::getPathToSource(){
	return &pathToSource;
}