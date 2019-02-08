#include "agent/task_handling/TaskHandler.h"

TaskHandler::TaskHandler(Agent* agent, ros::Publisher* scorePub, Map* map, MotionPlanner* mp, Gripper* gripper, ChargingManagement* cm, ReservationManager* rm) : 
	agent(agent),
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
	scoreMessage.robot_id = agent->getAgentID();
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
	scoreMessage.robot_id = agent->getAgentID();
	scoreMessage.request_id = requestId;
	scoreMessage.reject = true;
	scorePublisher->publish(scoreMessage);
}

void TaskHandler::update() {
	answerAnnouncements();
	if (!isTaskInExecution()) {
		if (isIdle()) {
			// check battery status,
			if(chargingManagement->isChargingAppropriate()) {
				double now = ros::Time::now().toSec();
				std::pair<Path, uint32_t> pathToCS = chargingManagement->getPathToNearestChargingStation(motionPlanner->getPositionAsOrientedPoint(), now);
				// add charging task
				if(pathToCS.first.isValid()) {
					ROS_WARN("[%s] Adding charging task while in idle state", agent->getAgentID().c_str());
					addChargingTask(pathToCS.second, pathToCS.first, now);
				} else {
					ROS_WARN("[%s] Could not add charging task while idling, path is invalid", agent->getAgentID().c_str());
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
					ROS_FATAL("[%s] Task is neither TransportationTask nor ChargingTask!", agent->getAgentID().c_str());
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
				motionPlanner->driveForward(0.5f);
			}
			break;

		case Task::State::PICKUP:
			if (motionPlanner->isDone()) {
				gripper->loadPackage(true);
				motionPlanner->driveBackward(0.5f);
				currentTask->setState(Task::State::RESERVING_TARGET);
				// reservationManager->startBiddingForPathReservation(motionPlanner->getPositionAsOrientedPoint(), currentTask->getTargetPosition(), TransportationTask::getDropOffTime());
			}
			break;

		case Task::State::RESERVING_TARGET:
			if (motionPlanner->isDone()) {
				if(reservationManager->isBidingForReservation()) {
					break;
				}
				if(reservationManager->getHasReservedPath() && hasTriedToReservePathToTarget){
					currentTask->setState(Task::State::TO_TARGET);
					motionPlanner->newPath(reservationManager->getReservedPath());
					motionPlanner->start();
				} else {
					// bid for a reservation if reservation failed
					reservationManager->startBiddingForPathReservation(motionPlanner->getPositionAsOrientedPoint(), currentTask->getTargetPosition(), TransportationTask::getDropOffTime());
					hasTriedToReservePathToTarget = true;
				}
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
					ROS_INFO("[%s] Starting charging", agent->getAgentID().c_str());
				}
				motionPlanner->driveForward(0.5f);
			}
			break;

		case Task::State::DROPOFF:
			if (motionPlanner->isDone()) {
				gripper->loadPackage(false);
				currentTask->setState(Task::State::LEAVE_TARGET);
				motionPlanner->driveBackward(0.5f);
			}
			break;

		case Task::State::CHARGING:
			// Check charging progress
			if (motionPlanner->isDone()) {
				if (this->chargingManagement->isCharged()) {
					ROS_INFO("[%s] Finished charging", agent->getAgentID().c_str());
					currentTask->setState(Task::State::LEAVE_TARGET);
					motionPlanner->driveBackward(0.5f);
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
			ROS_WARN("[STATE %s] Task FINISHED and should not be scheduled!", agent->getAgentID().c_str());
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
		if (queue.front()->isCharging()) {
			double now = ros::Time::now().toSec();
			std::pair<Path, uint32_t> pathToCS = chargingManagement->getPathToNearestChargingStation(motionPlanner->getPositionAsOrientedPoint(), now);
			if(pathToCS.first.isValid()) {
				((ChargingTask*) queue.front())->adjustChargingStation(pathToCS.second, pathToCS.first, now);
			} else {
				ROS_FATAL("[Task Handler %d] could not find a valid path to any charging Station when trying to start charging task", agent->getAgentIdInt());
				return;
			}
		}
		currentTask = queue.front();
		queue.pop_front();
		isNextTask = true;
		hasTriedToReservePathToTarget = false;
		
		
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

void TaskHandler::announcementCallback(const auto_smart_factory::TaskAnnouncement& tA) {
	announcements.push_back(tA);
}

void TaskHandler::answerAnnouncements() {
	while(!announcements.empty()) {
		auto_smart_factory::TaskAnnouncement tA = announcements.front();
		if(ros::Time::now() < tA.timeout){
			answerAnnouncement(tA);
		} else {
			ROS_INFO("[Task Handler %d] will not answer to Request %d as it already timeouted", agent->getAgentIdInt(), tA.request_id);
		}
		announcements.pop_front();
	}
}

void TaskHandler::answerAnnouncement(auto_smart_factory::TaskAnnouncement& taskAnnouncement) {
	double queuedDuration = getDuration();
	double estimatedBatteryAfterQueuedTasks = getEstimatedBatteryLevelAfterQueuedTasks();
	Task* lastTask = getLastTask();
	
	TrayScore* best = nullptr;

	for(uint32_t it_id : taskAnnouncement.start_ids){
		for(uint32_t st_id : taskAnnouncement.end_ids){
			// get Path
			auto_smart_factory::Tray input_tray = agent->getTray(it_id);
			auto_smart_factory::Tray storage_tray = agent->getTray(st_id);
			Path sourcePath;
			double startTime;
			
			if(lastTask != nullptr){
				// take the last position of the last task
				startTime = lastTask->getEndTime();
				sourcePath = map->getThetaStarPath(lastTask->getTargetPosition(), input_tray, startTime, TransportationTask::getPickUpTime());
			} else {
				// take the current position
				startTime = ros::Time::now().toSec();
				sourcePath = map->getThetaStarPath(agent->getCurrentOrientedPosition(), input_tray, startTime, TransportationTask::getPickUpTime());
			}
			
			if(!sourcePath.isValid()){
				continue;
			}
			
			Path targetPath = map->getThetaStarPath(input_tray, storage_tray, startTime + sourcePath.getDuration() + TransportationTask::getPickUpTime(), TransportationTask::getDropOffTime());
			
			if(!targetPath.isValid()) {
				continue;
			}
			
			double estimatedNewConsumption = sourcePath.getBatteryConsumption() + targetPath.getBatteryConsumption();

			// Check if task can be completed
			if(chargingManagement->isConsumptionPossible(estimatedBatteryAfterQueuedTasks, estimatedNewConsumption)) {
				double duration = queuedDuration + sourcePath.getDuration() + targetPath.getDuration();
				double scoreFactor = chargingManagement->getScoreMultiplierForBatteryLevel(estimatedBatteryAfterQueuedTasks - estimatedNewConsumption);
				double score = (1.f / scoreFactor) * duration;
				
				// add score to list
				double estimatedDuration = sourcePath.getDuration() + targetPath.getDuration();
				ROS_ASSERT_MSG(estimatedDuration > 0, "source-duration: %f | target-duration: %f", sourcePath.getDuration(), targetPath.getDuration());
				
				// Update best score
				if(best == nullptr || score < best->score){
					if(best != nullptr){
						delete best;
					}
					best = new TrayScore(it_id, st_id, score, estimatedDuration);
				}
			}
		}
	}
	
	if(best != nullptr) {
		// publish score
		publishScore(taskAnnouncement.request_id, best->score, best->sourceTray, best->targetTray, best->estimatedDuration);
		delete best;
	} else {
		// reject task
		rejectTask(taskAnnouncement.request_id);
		if(lastTask != nullptr && !(lastTask->isCharging()) ) {
			std::pair<Path, uint32_t> pathToCS = chargingManagement->getPathToNearestChargingStation(lastTask->getTargetPosition(), lastTask->getEndTime());
			// add charging task
			ROS_INFO("[Agent %d] Adding charging task because new task could not be taken", agent->getAgentIdInt());
			addChargingTask(pathToCS.second, pathToCS.first, lastTask->getEndTime());
		}
	}
}