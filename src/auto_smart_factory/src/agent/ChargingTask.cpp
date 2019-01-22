#include "agent/ChargingTask.h"

ChargingTask::ChargingTask(uint32_t targetID, OrientedPoint targetPos, Path targetPath) : 
	Task(targetID, targetPos, targetPath, Type::CHARGING){ 
	}

ChargingTask::~ChargingTask() = default;

float ChargingTask::getBatteryConsumption(void){
	return pathToTarget.getBatteryConsumption();
}

float ChargingTask::getDistance(void){
	return pathToTarget.getDistance();
}

void ChargingTask::setState(Task::State state) {
	if (state == Task::State::PICKUP || state == Task::State::DROPOFF || state == Task::State::TO_SOURCE) {
		return;
	}
	this->state = state;
}