#include "agent/task_handling/Task.h"

Task::Task(uint32_t targetId, Path pathToTarget, Type type, double startTime) : 
	targetId(targetId),
	type(type), 
	state(State::WAITING), startTime(startTime)
{
	targetDuration = pathToTarget.getDuration();
	targetBatCons = pathToTarget.getBatteryConsumption();
	targetPosition = pathToTarget.getEnd();
	startedAt = 0.0f;
	finishedAt = 0.0f;
	assignedAt = ros::Time::now().toSec();
	batteryStart = 0.0f;
	batteryFinish = 100.0f;
}

uint32_t Task::getTargetTrayId() {
	return targetId;
}

OrientedPoint Task::getTargetPosition() {
	return targetPosition;
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

double Task::getEndTime() {
	return startTime + this->getDuration();
}

void Task::setStartBatteryLevel(double batteryLevel) {
	if (this->state == Task::State::WAITING) {
		batteryStart = batteryLevel;
	}
}

void Task::setFinishBatteryLevel(double batteryLevel) {
	if (this->type == Task::Type::TRANSPORTATION && this->state == Task::State::FINISHED) {
		batteryFinish = batteryLevel;
	}
	if (this->type == Task::Type::CHARGING && this->state == Task::State::TO_TARGET && batteryLevel < batteryFinish) {
		batteryFinish = batteryLevel;
	}
}
