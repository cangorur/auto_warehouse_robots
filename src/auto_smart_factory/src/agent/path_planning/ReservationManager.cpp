
#include <include/agent/path_planning/ReservationManager.h>
#include <auto_smart_factory/ReservationRequest.h>

#include "agent/path_planning/ReservationManager.h"

ReservationManager::ReservationManager(ros::Publisher* publisher, Map* map, int agentId, Point startingPosition) :
	publisher(publisher),
	map(map),
	agentId(agentId),
	pathRetrievedCount(0),
	hasReservedPath(false),
	bidingForReservation(false),
	replanningNecessary(false),
	requestedEmergencyStop(false)
{
	// Add infinite reservation for starting point
	double now = ros::Time::now().toSec();
	double infiniteReservationStartTime = now - 1000.f;
	lastReservedPathReservations.emplace_back(startingPosition, Point(Path::getReservationSize(), Path::getReservationSize()), 0, infiniteReservationStartTime, Map::infiniteReservationTime, agentId);
}

void ReservationManager::update(Point pos) {
	double now = ros::Time::now().toSec();
	
	if(!replanningNecessary && !isInOwnReservation(pos, now)) {
		replanningNecessary = true;
	}
	
	map->deleteExpiredReservations(now);		
}

void ReservationManager::reservationBroadcastCallback(const auto_smart_factory::ReservationBroadcast& msg) {
	// This only works if the messages arrive in order
	if(msg.isReservationBroadcastOrDenial) {
		std::vector<Rectangle> oldReservations = map->deleteReservationsFromAgent(msg.ownerId);
		if(isReplanningBeneficialWithoutTheseReservations(oldReservations)) {
			replanningBeneficial = true;
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
				lastReservedPathTarget = pathToReserve.getEnd();
			}

			replanningNecessary = false;
			replanningBeneficial = false;
			saveReservationsAsLastReserved(msg);
			
		} else {
			if(msg.isEmergencyStop) {
				if(doesEmergencyStopPreventsOwnPath(reservations)) {
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

OrientedPoint ReservationManager::getLastReservedPathTarget() const {
	if(!hasReservedPath) {
		ROS_FATAL("[ReservationManager %d] Tried to get path target but hasNoReservedPath!", agentId);
	}
	
	return lastReservedPathTarget;
}

Path ReservationManager::getLastReservedPath() {
	if(!hasReservedPath) {
		ROS_FATAL("[ReservationManager %d] Tried to get path but hasNoReservedPath!", agentId);
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
		
		ROS_FATAL("[ReservationManager %d] Tried to generate path but no valid path was found from %f/%f to %f/%f", agentId, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
		ROS_WARN("Reservations for start:");
		map->listAllReservationsIn(Point(startPoint.x, startPoint.y));

		ROS_WARN("Reservations for target:");
		map->listAllReservationsIn(Point(endPoint.x, endPoint.y));
		return false;
	}
}

bool ReservationManager::isReplanningNecessary() const {
	return replanningNecessary;
}

bool ReservationManager::isInOwnReservation(Point pos, double time) {
	for(const Rectangle& r : lastReservedPathReservations) {
		if(r.getStartTime() <= time && time <= r.getEndTime() && Math::isPointInRectangle(pos, r)) {
			return true;
		}
	}
	
	return false;
}

bool ReservationManager::hasRequestedEmergencyStop() const {
	return requestedEmergencyStop; 
}

bool ReservationManager::doesEmergencyStopPreventsOwnPath(const std::vector<Rectangle>& emergencyStopReservations) const {
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
	double now = ros::Time::now().toSec();
	const std::vector<Point>& nodes = pathToReserve.getNodes();
	const std::vector<double>& departureTimes = pathToReserve.getDepartureTimes();

	for(int i = 0; i < nodes.size() - 1; i++) {
		if(now > departureTimes[i + 1]) {
			continue;
		}

		for(const Rectangle& oldReservation : oldReservations) {
			if(oldReservation.doesOverlapTimeRange(departureTimes[i], departureTimes[i + 1], agentId) && Math::doesLineSegmentIntersectRectangle(nodes[i], nodes[i + 1], oldReservation)) {
				return true;
			}
		}
	}

	return false;
}

bool ReservationManager::isReplanningBeneficial() const {
	return replanningBeneficial;
}
