#include "agent/ChargingTask.h"

ChargingTask::ChargingTask(uint32_t targetID, OrientedPoint targetPos, Path targetPath) : 
	Task(targetID, targetPos, targetPath, taskType::CHARGING){ 
	}

ChargingTask::~ChargingTask() = default;
