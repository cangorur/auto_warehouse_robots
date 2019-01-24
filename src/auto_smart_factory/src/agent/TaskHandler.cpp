#include "agent/TaskHandler.h"

TaskHandler::TaskHandler(std::string agentId, ros::Publisher* scorePub, Map* map, MotionPlanner* mp, Gripper* gripper, ChargingManagement* cm) 
        : 
    agentId(agentId),
    scorePublisher(scorePub),
    map(map),
    motionPlanner(mp),
    gripper(gripper),
    chargingManagement(cm)
{
}

void TaskHandler::publishScore(unsigned int requestId, double score, uint32_t startTrayId, uint32_t endTrayId) {
    // TODO: Score Calculation
    auto_smart_factory::TaskRating scoreMessage;
    scoreMessage.robot_id = agentId;
    scoreMessage.request_id = requestId;
    scoreMessage.score = score;
    scoreMessage.end_id = endTrayId;
    scoreMessage.start_id = startTrayId;
    scoreMessage.reject = false;
    scorePublisher->publish(scoreMessage);
}

void TaskHandler::rejectTask(unsigned int requestId) {
    auto_smart_factory::TaskRating scoreMessage;
    scoreMessage.robot_id = agentId;
    scoreMessage.request_id = requestId;
    scoreMessage.reject = true;
    scorePublisher->publish(scoreMessage);
}

void TaskHandler::update(void) {
    if (!isTaskInExecution()) {
        nextTask();
    } else {
        executeTask();
    }
}

// TODO: Delete queued Tasks & currentTask
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

void TaskHandler::executeTask(void) {
    if (isIdle()) {
        return;
    }

    switch(currentTask->getState()) {
        case Task::State::WAITING:
            if (currentTask->isTransportation()) {
                currentTask->setState(Task::State::TO_SOURCE);
                motionPlanner->newPath(((TransportationTask*) currentTask)->getPathToSource());
                this->motionPlanner->start();
            } else if (currentTask->isCharging()) {
                currentTask->setState(Task::State::TO_TARGET);
                motionPlanner->newPath(currentTask->getPathToTarget());
            }
            break;

        case Task::State::TO_SOURCE:
            if (this->motionPlanner->isDone()) {
                currentTask->setState(Task::State::PICKUP);
            }
            break;

        case Task::State::PICKUP:
            //if (gripper->loadPackage(true)) {
                currentTask->setState(Task::State::TO_TARGET);
                this->motionPlanner->newPath(currentTask->getPathToTarget());
                this->motionPlanner->start();
            //}
            break;

        case Task::State::TO_TARGET:
            // Check if target is reached
            // if yes:
            if (currentTask->isTransportation()) {
                if (motionPlanner->isDone()) {
                    currentTask->setState(Task::State::DROPOFF);
                }
            } else if (currentTask->isCharging()) {
                if (motionPlanner->isDone()) {
                    currentTask->setState(Task::State::CHARGING);
                    // activate charging
                }
            }
            break;

        case Task::State::DROPOFF:
            //if (gripper->loadPackage(false)) {
                currentTask->setState(Task::State::FINISHED);
                this->motionPlanner->stop();
            //}
            break;

        case Task::State::CHARGING:
            // Check charging progress
            // If new task in queue -> State = FINISHED
            // if finished -> State = FINISHED
            break;

        default:
            break;
    }
}

void TaskHandler::nextTask(void) {
    if (currentTask != nullptr) {
        delete currentTask;
    }
	if(!queue.empty()){
		currentTask = queue.front();
		queue.pop_front();
	} else {
        currentTask = nullptr;
    }
}

bool TaskHandler::isTaskInExecution(void) {
    return (currentTask != nullptr && currentTask->getState() != Task::State::FINISHED);
}

bool TaskHandler::isIdle(void) {
    return (currentTask == nullptr);
}

unsigned int TaskHandler::numberQueuedTasks(void) {
	return (unsigned int) queue.size();
}

Task* TaskHandler::getCurrentTask(void) {
    return currentTask;
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
