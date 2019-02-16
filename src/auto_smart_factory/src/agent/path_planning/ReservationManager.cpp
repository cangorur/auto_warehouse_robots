
#include <include/agent/path_planning/ReservationManager.h>
#include <auto_smart_factory/ReservationRequest.h>

#include "agent/path_planning/ReservationManager.h"

ReservationManager::ReservationManager(ros::Publisher* publisher, Map* map, int agentId, auto_smart_factory::WarehouseConfiguration warehouseConfig) :
	publisher(publisher),
	map(map),
	agentId(agentId),
	pathRetrievedCount(0),
	hasReservedPath(false),
	bidingForReservation(false),
	replanningNecessary(false),
	replanningBeneficial(false),
	requestedEmergencyStop(false)
{
	// Add infinite reservation for starting point
	double infiniteReservationStartTime = ros::Time::now().toSec() - 1000.f;


	// Add infinite reservation for starting point
	for(const auto& idlePosition : warehouseConfig.idle_positions) {
		std::string idStr = idlePosition.id.substr(idlePosition.id.find('_') + 1);
		int id = std::stoi(idStr);
		
		if(id == agentId) {
			Point pos = Point(static_cast<float>(idlePosition.pose.x), static_cast<float>(idlePosition.pose.y));
			lastReservedPathReservations.emplace_back(pos, Point(Path::getReservationSize(), Path::getReservationSize()), 0, infiniteReservationStartTime, Map::infiniteReservationTime, agentId);
		}
	}
}

void ReservationManager::update(Point pos) {
	double now = ros::Time::now().toSec();
	
	if(!replanningNecessary && !isInOwnReservation(pos, now)) {
		ROS_WARN("[RM %d] Replanning necessary because agent is not in own reservation", agentId);
		replanningNecessary = true;
	}
	
	map->deleteExpiredReservations(now);		
}

void ReservationManager::reservationBroadcastCallback(const auto_smart_factory::ReservationBroadcast& msg) {
	// This only works if the messages arrive in order
	if(msg.isReservationBroadcastOrDenial) {
		std::vector<Rectangle> oldReservations;
		if(!msg.isEmergencyStop) {
			oldReservations = map->deleteReservationsFromAgent(msg.ownerId);	
		}
						
		std::vector<Rectangle> reservations = getReservationsFromMessage(msg);
		map->addReservations(reservations);		
		
		if(msg.ownerId == agentId) {
			if(msg.isEmergencyStop) {
				requestedEmergencyStop = false;
			} else {
				hasReservedPath = true;
				bidingForReservation = false;
				pathRetrievedCount = 0;
			}

			replanningNecessary = false;
			replanningBeneficial = false;
			saveReservationsAsLastReserved(msg);
			
		} else {
			if(!replanningBeneficial && isReplanningBeneficialWithoutTheseReservations(oldReservations)) {
				ROS_WARN("[RM %d] Replanning beneficial because the path from robot %d was removed", agentId, msg.ownerId);
				replanningBeneficial = true;
			}
			
			if(msg.isEmergencyStop) {
				if(!replanningNecessary && doesEmergencyStopPreventsOwnPath(reservations)) {
					ROS_WARN("[RM %d] Replanning necessary because a emergency stop from robot %d appeared on its path", agentId, msg.ownerId);
					replanningNecessary = true;
				}
			}
		}
	} else if (msg.ownerId == agentId) {
		// Reservation was denied
		if(bidingForReservation) {
			if(calculateNewPath()) {
				requestPathReservation();
			}
		}		
	}	
}

std::vector<Rectangle> ReservationManager::getReservationsFromMessage(const auto_smart_factory::ReservationBroadcast& msg) {
	std::vector<Rectangle> reservations;
	for(auto r : msg.reservations) {
		reservations.emplace_back(Point(r.posX, r.posY), Point(r.sizeX, r.sizeY), r.rotation, r.startTime, r.endTime, r.ownerId);
	}
	
	return reservations;
}

void ReservationManager::publishEmergencyStop(Point pos) {
	requestedEmergencyStop = true;
	
	auto_smart_factory::ReservationRequest msg;
	msg.ownerId = agentId;
	msg.isEmergencyStop = static_cast<unsigned char>(true);
	double infiniteReservationStartTime = ros::Time::now().toSec() - 1000.f;
	
	auto_smart_factory::Rectangle rectangle;
	rectangle.posX = pos.x;
	rectangle.posY = pos.y;
	rectangle.sizeX = Path::getReservationSize();
	rectangle.sizeY = Path::getReservationSize();
	rectangle.rotation = 0;
	rectangle.startTime = infiniteReservationStartTime;
	rectangle.endTime = Map::infiniteReservationTime;
	rectangle.ownerId = agentId;

	msg.reservations.push_back(rectangle);
	publisher->publish(msg);
}

void ReservationManager::saveReservationsAsLastReserved(const auto_smart_factory::ReservationBroadcast& msg) {
	lastReservedPathReservations.clear();
	for(auto r : msg.reservations) {
		lastReservedPathReservations.emplace_back(Point(r.posX, r.posY), Point(r.sizeX, r.sizeY), r.rotation, r.startTime, r.endTime, r.ownerId);
	}
}

void ReservationManager::requestPathReservation() {
	if(pathToReserve.isValid()) {
		auto_smart_factory::ReservationRequest msg;
		msg.ownerId = agentId;
		msg.bid = pathToReserve.getDuration();
		msg.isEmergencyStop = static_cast<unsigned char>(false);

		for(const auto& r : pathToReserve.generateReservations(agentId)) {
			auto_smart_factory::Rectangle rectangle;
			rectangle.posX = r.getPosition().x;
			rectangle.posY = r.getPosition().y;
			rectangle.sizeX = r.getSize().x;
			rectangle.sizeY = r.getSize().y;
			rectangle.rotation = r.getRotation();
			rectangle.startTime = r.getStartTime();
			rectangle.endTime = r.getEndTime();
			rectangle.ownerId = r.getOwnerId();

			msg.reservations.push_back(rectangle);
		}

		publisher->publish(msg);	
	}	
}

void ReservationManager::startBiddingForPathReservation(OrientedPoint startPoint, OrientedPoint endPoint, double targetReservationDuration) {
	// Using Radiant here	
	this->startPoint = startPoint;
	this->endPoint = endPoint;	
	this->targetReservationDuration = targetReservationDuration;
	bidingForReservation = true;
	hasReservedPath = false;

	if(calculateNewPath()) {
		requestPathReservation();
	}
}

bool ReservationManager::isBidingForReservation() const {
	return bidingForReservation;
}

bool ReservationManager::getHasReservedPath() const {
	return hasReservedPath;
}

Path ReservationManager::getLastReservedPath() {
	if(!hasReservedPath) {
		ROS_FATAL("[RM %d] Tried to get path but hasNoReservedPath!", agentId);
	}
	pathRetrievedCount++;
	ROS_ASSERT_MSG(pathRetrievedCount == 1, "A reserved path may only be retrieved once!!");
	
	return pathToReserve;
}

bool ReservationManager::calculateNewPath() {
	pathToReserve = map->getThetaStarPath(startPoint, endPoint, ros::Time::now().toSec() + pathStartingTimeOffset, targetReservationDuration, true);

	if(pathToReserve.isValid()) {
		return true;
	} else {
		bidingForReservation = false;
		
		ROS_FATAL("[RM %d] Tried to generate path but no valid path was found from %f/%f to %f/%f", agentId, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
		//ROS_WARN("Reservations for start:");
		//map->listAllReservationsIn(Point(startPoint.x, startPoint.y));

		//ROS_WARN("Reservations for target:");
		//map->listAllReservationsIn(Point(endPoint.x, endPoint.y));
		return false;
	}
}

bool ReservationManager::isReplanningNecessary() const {
	return replanningNecessary;
}

bool ReservationManager::isInOwnReservation(Point pos, double time) {
	for(const Rectangle& r : lastReservedPathReservations) {
		if(r.getStartTime() - 0.5f <= time && time <= r.getEndTime() + 0.5f && Math::isPointInRectangle(pos, r)) {
			return true;
		}
	}
	
	return false;
}

bool ReservationManager::hasRequestedEmergencyStop() const {
	return requestedEmergencyStop; 
}

bool ReservationManager::doesEmergencyStopPreventsOwnPath(const std::vector<Rectangle>& emergencyStopReservations) const {
	if(!pathToReserve.isValid()) {
		return false;
	}
	
	double now = ros::Time::now().toSec();
	const std::vector<Point>& nodes = pathToReserve.getNodes();
	const std::vector<double>& departureTimes = pathToReserve.getDepartureTimes();
	
    for(int i = 0; i < nodes.size() - 1; i++) {
    	if(now > departureTimes[i + 1]) {
		    continue;
    	}
    	
    	for(const Rectangle& emergencyStop : emergencyStopReservations) {
		    if(emergencyStop.doesOverlapTimeRange(departureTimes[i], departureTimes[i + 1], agentId) && Math::doesLineSegmentIntersectRectangle(nodes[i], nodes[i + 1], emergencyStop)) {
				return true;
		    }
	    }
    }
    
    return false;
}

bool ReservationManager::isReplanningBeneficialWithoutTheseReservations(const std::vector<Rectangle>& oldReservations) const {
	if(!pathToReserve.isValid()) {
		return false;
	}

	for(const Rectangle& r : oldReservations) {
		if(Math::isPointInRectangle(Point(pathToReserve.getEnd().x, pathToReserve.getEnd().y), r)) {
			return true;
		}	
	}
	

	return false;
}

bool ReservationManager::isReplanningBeneficial() const {
	return replanningBeneficial;
}
