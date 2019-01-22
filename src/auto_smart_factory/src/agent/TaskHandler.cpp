#include "agent/TaskHandler.h"

TaskHandler::TaskHandler(std::string agentId, ros::Publisher* scorePub) 
        : 
    agentId(agentId),
    scorePublisher(scorePub)
{
}

void TaskHandler::publishScore(unsigned int requestId, double score, uint32_t startTrayId, uint32_t endTrayId) {
    auto_smart_factory::TaskRating scoreMessage;
    scoreMessage.robot_id = agentId;
    scoreMessage.request_id = requestId;
    scoreMessage.score = score;
    scoreMessage.end_id = endTrayId;
    scoreMessage.start_id = startTrayId;
    scoreMessage.reject = false;
    scorePublisher->publish(scoreMessage);
}

void TaskHandler::rejectTask(unsigned int requestId){
    auto_smart_factory::TaskRating scoreMessage;
    scoreMessage.robot_id = agentId;
    scoreMessage.request_id = requestId;
    scoreMessage.reject = true;
    scorePublisher->publish(scoreMessage);
}

TaskHandler::~TaskHandler() = default;

void TaskHandler::addTransportationTask(unsigned int id, uint32_t sourceID, OrientedPoint sourcePos, 
				uint32_t targetID, OrientedPoint targetPos, Path sourcePath, Path targetPath) {
    // create new task
    TransportationTask* t = new TransportationTask(id, sourceID, sourcePos, targetID, targetPos, sourcePath, targetPath);

    // add task to list
    queue.push_back(t);
}

void TaskHandler::addChargingTask(uint32_t targetID, OrientedPoint targetPos, Path targetPath) {
    // create new charging task
    ChargingTask* t = new ChargingTask(targetID, targetPos, targetPath);

    // add task to list
    queue.push_back(t);
}

void TaskHandler::nextTask(void) {
	if(!queue.empty()){
		currentTask = queue.front();
		queue.pop_front();
	}
}

unsigned int TaskHandler::numberQueuedTasks(void) {
	return (unsigned int) queue.size();
}

float TaskHandler::getBatteryConsumption(void) {
    float batteryCons = 0.0;
    for(std::list<Task*>::iterator t = queue.begin(); t != queue.end(); t++) {
        batteryCons += (*t)->getBatteryConsumption();
    }
    return batteryCons;
}

float TaskHandler::getDistance(void) {
    float distance = 0.0;
    for(std::list<Task*>::iterator t = queue.begin(); t != queue.end(); t++) {
        distance += (*t)->getDistance();
    }
    return distance;
}

Task* TaskHandler::getLastTask(void) {
    return queue.back();
}
