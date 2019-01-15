#include "agent/Task.h"

Task::Task(uint32_t targetId, OrientedPoint targetPosition, Path pathToTarget, taskType type) : 
	targetId(targetId), targetPosition(targetPosition), pathToTarget(pathToTarget), type(type){
}

uint32_t Task::getTargetTrayId(void){
	return targetId;
}

OrientedPoint Task::getTargetPosition(void){
	return targetPosition;
}

Path* Task::getPathToTarget(void){
	return &pathToTarget;
}

Task::taskType Task::getType(void){
	return this->type;
}