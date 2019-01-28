#include "agent/task_handling/TaskHandler.h"

TaskHandler::TaskHandler(std::string agentId, ros::Publisher* scorePub, Map* map, MotionPlanner* mp, Gripper* gripper, ChargingManagement* cm, ReservationManager* rm) 
        : 
    agentId(agentId),
    scorePublisher(scorePub),
    map(map),
    motionPlanner(mp),
    gripper(gripper),
    chargingManagement(cm),
    reservationManager(rm)
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

void TaskHandler::rejectTask(unsigned int requestId) {
    auto_smart_factory::TaskRating scoreMessage;
    scoreMessage.robot_id = agentId;
    scoreMessage.request_id = requestId;
    scoreMessage.reject = true;
    scorePublisher->publish(scoreMessage);
}

void TaskHandler::update() {
    if (!isTaskInExecution()) {
        nextTask();
    } else {
        executeTask();
    }
}

TaskHandler::~TaskHandler(){
    if (currentTask != nullptr) {
        delete currentTask;
    }
    for(Task* t : queue){
        delete t;
    }
    queue.clear();
}

void TaskHandler::addTransportationTask(unsigned int id, uint32_t sourceID, OrientedPoint sourcePos, 
				uint32_t targetID, OrientedPoint targetPos, Path sourcePath, Path targetPath, double startTime) {
    // create new task
    TransportationTask* t = new TransportationTask(id, sourceID, sourcePos, targetID, targetPos, sourcePath, targetPath, startTime);

    // add task to list
    queue.push_back(t);
}

void TaskHandler::addChargingTask(uint32_t targetID, OrientedPoint targetPos, Path targetPath, double startTime) {
    // create new charging task
    ChargingTask* t = new ChargingTask(targetID, targetPos, targetPath, startTime);

    // add task to list
    queue.push_back(t);
}

void TaskHandler::executeTask() {
    if (isIdle()) {
        return;
    }

    switch(currentTask->getState()) {
        case Task::State::WAITING:
            if(reservationManager->getIsBidingForReservation()) {
                break;
            }
            
            if(reservationManager->getHasReservedPath()) {
                if (currentTask->isTransportation()) {
                    currentTask->setState(Task::State::TO_SOURCE);
                    this->motionPlanner->start();
                } else if (currentTask->isCharging()) {
                    currentTask->setState(Task::State::TO_TARGET);
                }
                motionPlanner->newPath(reservationManager->getReservedPath());
                
            } else {
                // Start to bid for path reservations
                Path* pathBlueprint;
                
                if(currentTask->isTransportation()) {
                    pathBlueprint = ((TransportationTask*) currentTask)->getPathToSource();
                } else if(currentTask->isCharging()) {
                    pathBlueprint = currentTask->getPathToTarget();
                } else {
                    ROS_ERROR("Task is neither Transportation task nor charging task!");
                }
                
                reservationManager->bidForPathReservation(pathBlueprint->getNodes().front(), pathBlueprint->getNodes().back());
            }           
            
            break;

        case Task::State::TO_SOURCE:
            if (this->motionPlanner->isDone()) {
                currentTask->setState(Task::State::PICKUP);
                motionPlanner->alignTowards(((TransportationTask*) currentTask)->getSourcePosition().o);
            }
            break;

        case Task::State::PICKUP:
            if (this->motionPlanner->isDone()) {
                gripper->loadPackage(true);
                ros::Duration(2).sleep();
                currentTask->setState(Task::State::TO_TARGET);
                this->motionPlanner->newPath(currentTask->getPathToTarget());
                this->motionPlanner->start();
            }
            break;

        case Task::State::TO_TARGET:
            if (currentTask->isTransportation()) {
                if (motionPlanner->isDone()) {
                    currentTask->setState(Task::State::DROPOFF);
                    motionPlanner->alignTowards(currentTask->getTargetPosition().o);
                }
            } else if (currentTask->isCharging()) {
                if (motionPlanner->isDone()) {
                    currentTask->setState(Task::State::CHARGING);
                    motionPlanner->alignTowards(currentTask->getTargetPosition().o);
                    // activate charging
                }
            }
            break;

        case Task::State::DROPOFF:
            if (motionPlanner->isDone()) {
                gripper->loadPackage(false);
                currentTask->setState(Task::State::FINISHED);
                this->motionPlanner->stop();
            }
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

void TaskHandler::nextTask() {
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

bool TaskHandler::isTaskInExecution() {
    return (currentTask != nullptr && currentTask->getState() != Task::State::FINISHED);
}

bool TaskHandler::isIdle() {
    return (currentTask == nullptr);
}

unsigned int TaskHandler::numberQueuedTasks() {
	return (unsigned int) queue.size();
}

Task* TaskHandler::getCurrentTask() {
    return currentTask;
}

float TaskHandler::getBatteryConsumption() {
    float batteryCons = 0.0;
    for(Task* t : queue) {
        batteryCons += t->getBatteryConsumption();
    }
    if(currentTask != nullptr) {
        batteryCons += currentTask->getBatteryConsumption();
    }
    return batteryCons;
}

float TaskHandler::getDistance() {
    float distance = 0.0;
    for(Task* t : queue) {
        distance += t->getDistance();
    }
    if(currentTask != nullptr) {
        distance += currentTask->getDistance();
    }
    return distance;
}

double TaskHandler::getDuration() {
    double duration = 0.0;
    for(Task* t : queue) {
        duration += t->getDuration();
    }
    if(currentTask != nullptr) {
        duration += currentTask->getDuration();
    }
    return duration;
}

Task* TaskHandler::getLastTask() {
    if(!queue.empty()){
        return queue.back();
    } else if(currentTask != nullptr) {
        return currentTask;
    } else {
        return nullptr;
    }
}
