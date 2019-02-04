
#include <include/agent/task_handling/ChargingTask.h>

#include "agent/task_handling/ChargingTask.h"

double ChargingTask::chargingTime = 40.f;

ChargingTask::ChargingTask(uint32_t targetID, Path targetPath, double startTime) : 
	Task(targetID, targetPath, Type::CHARGING, startTime)
{
}

double ChargingTask::getBatteryConsumption() {
	if( state == Task::State::WAITING || state == Task::State::TO_TARGET) {
		return targetBatCons;
	} else {
		return 0.0;
	}
}

double ChargingTask::getDuration() {
	if( state == Task::State::WAITING || state == Task::State::TO_TARGET) {
		return targetDuration + chargingTime;
	} else if (state == Task::State::CHARGING) {
		return chargingTime;
	} 
	return 0.0;
}

void ChargingTask::setState(Task::State state) {
	if (state == Task::State::PICKUP || state == Task::State::DROPOFF || state == Task::State::TO_SOURCE) {
		return;
	}
	this->state = state;
}

double ChargingTask::getChargingTime() {
	return chargingTime;
}
