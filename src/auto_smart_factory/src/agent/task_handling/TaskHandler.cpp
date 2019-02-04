#include "agent/task_handling/TaskHandler.h"

TaskHandler::TaskHandler(std::string agentId, ros::Publisher* scorePub, Map* map, MotionPlanner* mp, Gripper* gripper, ChargingManagement* cm, ReservationManager* rm) : 
    agentId(agentId),
    scorePublisher(scorePub),
    map(map),
    motionPlanner(mp),
    gripper(gripper),
    chargingManagement(cm),
    reservationManager(rm)
{
}

void TaskHandler::publishScore(unsigned int requestId, double score, uint32_t startTrayId, uint32_t endTrayId, double estimatedDuration) {
    auto_smart_factory::TaskRating scoreMessage;
    scoreMessage.robot_id = agentId;
    scoreMessage.request_id = requestId;
    scoreMessage.score = score;
    ROS_ASSERT_MSG(estimatedDuration > 0, "Published Score with estimatedDuration == 0");
    scoreMessage.estimatedDuration = estimatedDuration;
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
            if(chargingManagement->isChargingAppropriate()) {
                double now = ros::Time::now().toSec();
                std::pair<Path, uint32_t> pathToCS = chargingManagement->getPathToNearestChargingStation(motionPlanner->getPositionAsOrientedPoint(), now);
                // add charging task
                if(pathToCS.first.isValid()) {
                    ROS_WARN("[%s] Adding charging task while in idle state", agentId.c_str());
                    addChargingTask(pathToCS.second, pathToCS.first, now);
                } else {
                    ROS_WARN("[%s] Could not add charging task while idling, path is invalid", agentId.c_str());
                }
            }
        }
        nextTask();
    } else {
        executeTask();
    }
}

TaskHandler::~TaskHandler() {
    if (currentTask != nullptr) {
        delete currentTask;
    }
    for(Task* t : queue){
        delete t;
    }
    queue.clear();
}

void TaskHandler::addTransportationTask(unsigned int id, uint32_t sourceID, uint32_t targetID, Path sourcePath, Path targetPath, double startTime) {
    if(sourcePath.isValid() && targetPath.isValid()) {
        // create new task
        TransportationTask* t = new TransportationTask(id, sourceID, targetID, sourcePath, targetPath, startTime);

        // add task to list
        queue.push_back(t);
    }
}

void TaskHandler::addChargingTask(uint32_t targetID, Path targetPath, double startTime) {
    if(targetPath.isValid()) {
        // create new charging task
        ChargingTask* t = new ChargingTask(targetID, targetPath, startTime);

        // add task to list
        queue.push_back(t);
    }
}

void TaskHandler::executeTask() {
    if(isIdle()) {
        return;
    }

    switch(currentTask->getState()) {
        case Task::State::WAITING:
            if(reservationManager->isBidingForReservation()) {
                break;
            }
            
            if(reservationManager->getHasReservedPath() && !isNextTask) {
                if (currentTask->isTransportation()) {
                    currentTask->setState(Task::State::TO_SOURCE);
                } else if (currentTask->isCharging()) {
                    currentTask->setState(Task::State::TO_TARGET);
                }
                motionPlanner->newPath(reservationManager->getReservedPath());
                motionPlanner->start();
            } else {
                // Start to bid for path reservations
                if(currentTask->isTransportation()) {
	                reservationManager->startBiddingForPathReservation(motionPlanner->getPositionAsOrientedPoint(), ((TransportationTask*) currentTask)->getSourcePosition(), TransportationTask::getPickUpTime());
                } else if(currentTask->isCharging()) {
	                reservationManager->startBiddingForPathReservation(motionPlanner->getPositionAsOrientedPoint(), currentTask->getTargetPosition(), ChargingTask::getChargingTime());
                } else {
                    ROS_FATAL("[%s] Task is neither TransportationTask nor ChargingTask!", agentId.c_str());
                }
                isNextTask = false;
            }
            break;

        case Task::State::TO_SOURCE:
            if (motionPlanner->isDone()) {
                currentTask->setState(Task::State::APPROACH_SOURCE);
                motionPlanner->alignTowards(((TransportationTask*) currentTask)->getSourcePosition().o);
            }
            break;

        case Task::State::APPROACH_SOURCE:
            if (motionPlanner->isDone()) {
                currentTask->setState(Task::State::PICKUP);
                motionPlanner->driveForward(0.3f);
            }
            break;

        case Task::State::PICKUP:
            if (motionPlanner->isDone()) {
                gripper->loadPackage(true);
                //ros::Duration(0.1f).sleep();
                motionPlanner->driveBackward(0.3f);
                currentTask->setState(Task::State::LEAVE_SOURCE);
            }
            break;

        case Task::State::LEAVE_SOURCE:
            if (motionPlanner->isDone()) {
                // TODO Error if no path found -> check of path can be found if not -> drive away anyway
	            reservationManager->startBiddingForPathReservation(motionPlanner->getPositionAsOrientedPoint(), currentTask->getTargetPosition(), TransportationTask::getDropOffTime());
                currentTask->setState(Task::State::RESERVING_TARGET);
            }
            break;

        case Task::State::RESERVING_TARGET:
            if(reservationManager->isBidingForReservation()) {
                break;
            }
            if(reservationManager->getHasReservedPath()) {
                currentTask->setState(Task::State::TO_TARGET);
                motionPlanner->newPath(reservationManager->getReservedPath());
                motionPlanner->start();   
            }
            break;

        case Task::State::TO_TARGET:
            if (motionPlanner->isDone()) {
                currentTask->setState(Task::State::APPROACH_TARGET);
                motionPlanner->alignTowards(currentTask->getTargetPosition().o);
            }
            break;

        case Task::State::APPROACH_TARGET:
            if (motionPlanner->isDone()) {
                if (currentTask->isTransportation()) {
                    currentTask->setState(Task::State::DROPOFF);
                } else if (currentTask->isCharging()) {
                    currentTask->setState(Task::State::CHARGING);
                    ROS_INFO("[%s] Starting charging", agentId.c_str());
                }
                motionPlanner->driveForward(0.3f);
            }
            break;

        case Task::State::DROPOFF:
            if (motionPlanner->isDone()) {
                gripper->loadPackage(false);
                //ros::Duration(2).sleep();
                currentTask->setState(Task::State::LEAVE_TARGET);
                motionPlanner->driveBackward(0.3f);
            }
            break;

        case Task::State::CHARGING:
            // Check charging progress
            if (motionPlanner->isDone()) {
                if (this->chargingManagement->isCharged()) {
                    ROS_INFO("[%s] Finished charging", agentId.c_str());
                    currentTask->setState(Task::State::LEAVE_TARGET);
                    motionPlanner->driveBackward(0.3f);
                }
            }
            break;

        case Task::State::LEAVE_TARGET:
            if (motionPlanner->isDone()) {
                motionPlanner->stop();
                currentTask->setState(Task::State::FINISHED);
            }
            break;

        case Task::State::FINISHED:
            /* Wait for next task */
            ROS_WARN("[STATE %s] Task FINISHED and should not be scheduled!", agentId.c_str());
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
        isNextTask = true;
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
