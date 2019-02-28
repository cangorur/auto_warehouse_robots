#include "agent/task_handling/TransportationTask.h"

double TransportationTask::pickUpTime = 50.f;
double TransportationTask::dropOffTime = 50.f;

TransportationTask::TransportationTask(unsigned int id, uint32_t sourceID, uint32_t targetID, Path sourcePath, Path targetPath, double startTime) : 
	Task(targetID, targetPath, Type::TRANSPORTATION, startTime),
	id(id), 
	sourceId(sourceID)
{
		sourceDuration = sourcePath.getDuration();
		sourceBatCons = sourcePath.getBatteryConsumption();
		sourcePosition = sourcePath.getEnd();
		startedPickUpAt = 0.0f;
		finishedPickUpAt = 0.0f;
		startedDropOffAt = 0.0f;
}

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
	if(state == Task::State::WAITING || state == Task::State::TO_SOURCE || state == Task::State::APPROACH_SOURCE) {
		return sourceBatCons + targetBatCons;
	} else if(state == Task::State::PICKUP || state == Task::State::RESERVING_TARGET || state == Task::State::TO_TARGET) {
		return targetBatCons;
	}
	return 0.0;
}

double TransportationTask::getDuration(){
	if(state == Task::State::WAITING || state == Task::State::TO_SOURCE || state == Task::State::APPROACH_SOURCE) {
		return sourceDuration + pickUpTime + targetDuration + dropOffTime;
	} else if(state == Task::State::PICKUP) {
		return pickUpTime + targetDuration + dropOffTime;
	} else if(state == Task::State::RESERVING_TARGET || state == Task::State::TO_TARGET || state == Task::State::APPROACH_TARGET) {
		return targetDuration + dropOffTime;
	} else if(state == Task::State::DROPOFF || state == Task::State::LEAVE_TARGET) {
		return dropOffTime;
	}
	return 0.0;
}

void TransportationTask::setState(Task::State state) {
	if (state == Task::State::CHARGING) {
		return;
	}
	switch (state) {
		case Task::State::TO_SOURCE:
			if(startedAt == 0.0f) {
				startedAt = ros::Time::now().toSec();
			}
			break;
		
		case Task::State::APPROACH_SOURCE:
			if(startedPickUpAt == 0.0f) {
				startedPickUpAt = ros::Time::now().toSec();
			}
			break;
		
		case Task::State::TO_TARGET:
			if(finishedPickUpAt == 0.0f) {
				finishedPickUpAt = ros::Time::now().toSec();
			}
			break;

		case Task::State::APPROACH_TARGET:
			if(startedDropOffAt == 0.0f) {
				startedDropOffAt = ros::Time::now().toSec();
			}
			break;

		case Task::State::FINISHED:
			if(finishedAt == 0.0f) {
				finishedAt = ros::Time::now().toSec();
			}
			break;
	
		default:
			break;
	}
	this->state = state;
}

double TransportationTask::getPickUpTime() {
	return pickUpTime;
}

double TransportationTask::getDropOffTime() {
	return dropOffTime;
}

void TransportationTask::fillInEvaluationData(auto_smart_factory::TaskEvaluation* msg) {
	msg->task_type = "transportation";
	msg->finishedAt = finishedAt;
	msg->assignedAt = assignedAt;
	msg->startedAt = startedAt;
	msg->request_id = id;
	msg->startedPickUpAt = startedPickUpAt;
	msg->finishedPickUpAt = finishedPickUpAt;
	msg->startedDropOffAt = startedDropOffAt;
}
