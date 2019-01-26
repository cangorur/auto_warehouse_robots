#include "agent/Task.h"

Task::Task(uint32_t targetId, OrientedPoint targetPosition, Path pathToTarget, Type type, double startTime) : 
	targetId(targetId), targetPosition(targetPosition), pathToTarget(pathToTarget), type(type), 
	state(State::WAITING), startTime(startTime) {
}

uint32_t Task::getTargetTrayId() {
	return targetId;
}

OrientedPoint Task::getTargetPosition() {
	return targetPosition;
}

Path* Task::getPathToTarget() {
	return &pathToTarget;
}

Task::Type Task::getType() {
	return this->type;
}

Task::State Task::getState() {
	return this->state;
}

bool Task::isTransportation() {
	return (this->type == Task::Type::TRANSPORTATION);
}

bool Task::isCharging() {
	return (this->type == Task::Type::CHARGING);
}

double Task::getEndTime(void) {
	return startTime + this->getDuration();
}