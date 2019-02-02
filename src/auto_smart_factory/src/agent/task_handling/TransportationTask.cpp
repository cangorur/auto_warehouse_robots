#include "agent/task_handling/TransportationTask.h"

TransportationTask::TransportationTask(unsigned int id, uint32_t sourceID, uint32_t targetID, 
		Path sourcePath, Path targetPath, double startTime) : 
	Task(targetID, targetPath, Type::TRANSPORTATION, startTime),
	id(id), 
	sourceId(sourceID)
{
		sourceDuration = sourcePath.getDuration();
		sourceBatCons = sourcePath.getBatteryConsumption();
		sourcePosition = sourcePath.getEnd();
}

TransportationTask::~TransportationTask() = default;

unsigned int TransportationTask::getId(){
	return id;
}

uint32_t TransportationTask::getSourceTrayId(){
	return sourceId;
}

OrientedPoint TransportationTask::getSourcePosition(){
	return sourcePosition;
}

double TransportationTask::getBatteryConsumption(){
	if(state == Task::State::WAITING || state == Task::State::TO_SOURCE) {
		return sourceBatCons + targetBatCons;
	} else if(state == Task::State::PICKUP || state == Task::State::TO_TARGET || state == Task::State::RESERVING_TARGET) {
		return targetBatCons;
	}
	return 0.0;
}

double TransportationTask::getDuration(){
	if(state == Task::State::WAITING || state == Task::State::TO_SOURCE) {
		return sourceDuration + pickUpTime + targetDuration + dropOffTime;
	} else if(state == Task::State::PICKUP) {
		return pickUpTime + targetDuration + dropOffTime;
	} else if(state == Task::State::TO_TARGET || state == Task::State::RESERVING_TARGET) {
		return targetDuration + dropOffTime;
	} else if(state == Task::State::DROPOFF) {
		return dropOffTime;
	}
	return 0.0;
}

void TransportationTask::setState(Task::State state) {
	if (state == Task::State::CHARGING) {
		return;
	}
	this->state = state;
}