#include "agent/Task.h"

Task::Task():
	id(-1), 
	sourceId(0),
	targetId(0),
	sourcePosition(OrientedPoint(-1, -1, -1)),
	targetPosition(OrientedPoint(-1, -1, -1)),
	pathToSource(Path({})), 
	pathToTarget(Path({})){
}

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