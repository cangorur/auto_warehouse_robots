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
        if (isIdle()) {
            // check battery status,
            if(this->chargingManagement->isChargingAppropriate()) {
                double now = ros::Time::now().toSec();
                std::pair<Path, uint32_t> pathToCS = this->chargingManagement->getPathToNearestChargingStation(this->motionPlanner->getOrientedPoint(), now);
                // add charging task
                if(pathToCS.first.getDistance() > 0) {
                    ROS_WARN("[%s] adding charging task while in idle state", agentId.c_str());
                    addChargingTask(pathToCS.second, pathToCS.first, now);
                } else {
                    ROS_WARN("[%s] could not add charging task while idling, because its length is 0", agentId.c_str());
                }
            }
        }
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

void TaskHandler::addTransportationTask(unsigned int id, uint32_t sourceID, uint32_t targetID, 
				Path sourcePath, Path targetPath, double startTime) {
    if(sourcePath.getDistance() > 0 && targetPath.getDistance() > 0){
        // create new task
        TransportationTask* t = new TransportationTask(id, sourceID, targetID, sourcePath, targetPath, startTime);

        // add task to list
        queue.push_back(t);
    }
}

void TaskHandler::addChargingTask(uint32_t targetID, Path targetPath, double startTime) {
    if(targetPath.getDistance() > 0){
        // create new charging task
        ChargingTask* t = new ChargingTask(targetID, targetPath, startTime);

        // add task to list
        queue.push_back(t);
    }
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
                if(currentTask->isTransportation()) {
                    reservationManager->bidForPathReservation(motionPlanner->getOrientedPoint(), ((TransportationTask*) currentTask)->getSourcePosition());
                } else if(currentTask->isCharging()) {
                    reservationManager->bidForPathReservation(motionPlanner->getOrientedPoint(), currentTask->getTargetPosition());
                } else {
                    ROS_ERROR("Task is neither Transportation task nor charging task!");
                }
            }           
            
            break;

        case Task::State::TO_SOURCE:
            if (this->motionPlanner->isDone()) {
                currentTask->setState(Task::State::PICKUP);
                motionPlanner->alignTowards(((TransportationTask*) currentTask)->getSourcePosition().o);
                this->motionPlanner->stop();
            }
            break;

        case Task::State::PICKUP:
            if (this->motionPlanner->isDone()) { 
                gripper->loadPackage(true);
                reservationManager->bidForPathReservation(((TransportationTask*) currentTask)->getSourcePosition(), currentTask->getTargetPosition());
                ros::Duration(2).sleep();
                currentTask->setState(Task::State::RESERVING_TARGET);
            }
            break;

        case Task::State::RESERVING_TARGET:
            if(reservationManager->getIsBidingForReservation()) {
                break;
            }
            if(reservationManager->getHasReservedPath()) {
                currentTask->setState(Task::State::TO_TARGET);
                motionPlanner->newPath(reservationManager->getReservedPath());
                this->motionPlanner->start();   
            }
            break;

        case Task::State::TO_TARGET:
            if (currentTask->isTransportation()) {
                if (motionPlanner->isDone()) {
                    currentTask->setState(Task::State::DROPOFF);
                    motionPlanner->alignTowards(currentTask->getTargetPosition().o);
                    this->motionPlanner->stop();
                }
            } else if (currentTask->isCharging()) {
                if (motionPlanner->isDone()) {
                    currentTask->setState(Task::State::CHARGING);
                    motionPlanner->alignTowards(currentTask->getTargetPosition().o);
                    this->motionPlanner->stop();
                }
            }
            break;

        case Task::State::DROPOFF:
            if (motionPlanner->isDone()) {
                gripper->loadPackage(false);
                currentTask->setState(Task::State::FINISHED);
            }
            break;

        case Task::State::CHARGING:
            // Check charging progress
            this->motionPlanner->stop();
            if (this->chargingManagement->isCharged()){
                currentTask->setState(Task::State::FINISHED);
            }
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

double TaskHandler::getEstimatedBatteryLevelAfterQueuedTasks() {
	double estimatedBattery = chargingManagement->getCurrentBattery();

    if(currentTask != nullptr) {
        if(currentTask->isCharging()) {
            estimatedBattery = 99.f;
        } else {
            estimatedBattery -= currentTask->getBatteryConsumption();
        }
    }	
    for(Task* t : queue) {
        if(t->isCharging()) {
            estimatedBattery = 99.f;
        } else {
            estimatedBattery -= t->getBatteryConsumption();
        }
    }
 
    return estimatedBattery;
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
