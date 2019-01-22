#include "agent/TransportationTask.h"

TransportationTask::TransportationTask(unsigned int id, uint32_t sourceID, OrientedPoint sourcePos, 
			uint32_t targetID, OrientedPoint targetPos, Path sourcePath, Path targetPath) : 
	Task(targetID, targetPos, targetPath, Type::TRANSPORTATION),
	id(id), 
	sourceId(sourceID),
	sourcePosition(sourcePos),
	pathToSource(sourcePath) {
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

Path* TransportationTask::getPathToSource(){
	return &pathToSource;
}

float TransportationTask::getBatteryConsumption(){
	return pathToSource.getBatteryConsumption() + pathToTarget.getBatteryConsumption();
}

float TransportationTask::getDistance(){
	return pathToSource.getDistance() + pathToTarget.getDistance();
}

void TransportationTask::setState(Task::State state) {
	if (state == Task::State::CHARGING) {
		return;
	}
	this->state = state;
}