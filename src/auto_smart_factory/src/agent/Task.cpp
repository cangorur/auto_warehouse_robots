#include "agent/Task.h"

Task::Task(uint32_t targetId, OrientedPoint targetPosition, Path pathToTarget, Type type) : 
	targetId(targetId), targetPosition(targetPosition), pathToTarget(pathToTarget), type(type), state(State::WAITING) {
}

uint32_t Task::getTargetTrayId(void) {
	return targetId;
}

OrientedPoint Task::getTargetPosition(void) {
	return targetPosition;
}

Path* Task::getPathToTarget(void) {
	return &pathToTarget;
}

Task::Type Task::getType(void) {
	return this->type;
}

Task::State Task::getState(void) {
	return this->state;
}

bool Task::isTransportation(void) {
	return (this->type == Task::Type::TRANSPORTATION);
}

bool Task::isCharging(void) {
	return (this->type == Task::Type::CHARGING);
}