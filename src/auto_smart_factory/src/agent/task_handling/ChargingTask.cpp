#include "agent/task_handling/ChargingTask.h"

ChargingTask::ChargingTask(uint32_t targetID, OrientedPoint targetPos, Path targetPath, double startTime) : 
	Task(targetID, targetPos, targetPath, Type::CHARGING, startTime){ 
	}

ChargingTask::~ChargingTask() = default;

float ChargingTask::getBatteryConsumption(void){
	return pathToTarget.getBatteryConsumption();
}

float ChargingTask::getDistance(void){
	return pathToTarget.getDistance();
}

double ChargingTask::getDuration(void){
	return pathToTarget.getDuration() + chargingTime;
}

void ChargingTask::setState(Task::State state) {
	if (state == Task::State::PICKUP || state == Task::State::DROPOFF || state == Task::State::TO_SOURCE) {
		return;
	}
	this->state = state;
}