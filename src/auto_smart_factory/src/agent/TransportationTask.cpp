#include "agent/TransportationTask.h"

TransportationTask::TransportationTask(unsigned int id, uint32_t sourceID, OrientedPoint sourcePos, 
			uint32_t targetID, OrientedPoint targetPos, Path sourcePath, Path targetPath) : 
	Task(targetID, targetPos, targetPath, taskType::TRANSPORTATION),
	id(id), 
	sourceId(sourceID),
	sourcePosition(sourcePos),
	pathToSource(sourcePath){
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

float TransportationTask::getBatteryConsumption(void){
	// TODO: add battery consumption function for pathes
	return 0.0;
}

float TransportationTask::getDistance(void){
	return pathToSource.getLength() + pathToTarget.getLength();
}