#include "agent/task_handling/ChargingTask.h"

double ChargingTask::chargingTime = 40.f;

ChargingTask::ChargingTask(uint32_t targetID, Path targetPath, double startTime) : 
	Task(targetID, targetPath, Type::CHARGING, startTime)
{
}

double ChargingTask::getBatteryConsumption() {
	if( state == Task::State::WAITING || state == Task::State::TO_TARGET || state == Task::State::APPROACH_TARGET || state == Task::State::LEAVE_TARGET) {
		return targetBatCons;
	} else {
		return 0.0;
	}
}

double ChargingTask::getDuration() {
	if( state == Task::State::WAITING || state == Task::State::TO_TARGET || state == Task::State::APPROACH_TARGET || state == Task::State::LEAVE_TARGET) {
		return targetDuration + chargingTime;
	} else if (state == Task::State::CHARGING) {
		return chargingTime;
	} 
	return 0.0;
}

void ChargingTask::setState(Task::State state) {
	if (state == Task::State::TO_SOURCE || state == Task::State::APPROACH_SOURCE || state == Task::State::PICKUP || state == Task::State::RESERVING_TARGET || state == Task::State::DROPOFF) {
		return;
	}
	switch (state) {
		case Task::State::TO_TARGET:
			startedAt = ros::Time::now().toSec();
			break;
		
		case Task::State::CHARGING:
			arrivedAt = ros::Time::now().toSec();
			break;
		
		case Task::State::FINISHED:
			finishedAt = ros::Time::now().toSec();
			ROS_INFO("Charging Task execution started at %f with duration %f\n\tTook %f to drive to Charging Station\n\tTook %f to charge", 
				startedAt, (finishedAt-startedAt), (arrivedAt-startedAt), (finishedAt-arrivedAt));
			break;
	
		default:
			break;
	}
	this->state = state;
}

double ChargingTask::getChargingTime() {
	return chargingTime;
}

void ChargingTask::adjustChargingStation(uint32_t targetID, Path targetPath, double startTime){
	this->targetId = targetID;
	this->startTime = startTime;
	this->targetDuration = targetPath.getDuration();
	this->targetBatCons = targetPath.getBatteryConsumption();
	this->targetPosition = targetPath.getEnd();
}
