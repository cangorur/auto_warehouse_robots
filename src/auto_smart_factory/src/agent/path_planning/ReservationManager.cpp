
#include <include/agent/path_planning/ReservationManager.h>
#include <auto_smart_factory/ReservationRequest.h>

#include "agent/path_planning/ReservationManager.h"

ReservationManager::ReservationManager(ros::Publisher* publisher, Map* map, int agentId) :
	publisher(publisher),
	map(map),
	agentId(agentId),
	pathRetrievedCount(0),
	hasReservedPath(false),
	bidingForReservation(false)
{
}

void ReservationManager::update() {
	double now = ros::Time::now().toSec();
	map->deleteExpiredReservations(now);		
}

void ReservationManager::reservationBroadcastCallback(const auto_smart_factory::ReservationBroadcast& msg) {
	// This only works if the messages arrive in order
	
	if(msg.isReservationBroadcastOrDenial) {
		if(msg.ownerId == agentId) {
			hasReservedPath = true;
			bidingForReservation = false;
			pathRetrievedCount = 0;
		}
		map->deleteReservationsFromAgent(msg.ownerId);
		addReservations(msg);
	} else if (msg.ownerId == agentId) {
		// Reservation was denied
		
		if(bidingForReservation) {
			if(calculateNewPath()) {
				requestPathReservation();
			}
		}		
	}	
}

void ReservationManager::addReservations(const auto_smart_factory::ReservationBroadcast& msg) {
	std::vector<Rectangle> reservations;
	
	for(auto r : msg.reservations) {
		reservations.emplace_back(Point(r.posX, r.posY), Point(r.sizeX, r.sizeY), r.rotation, r.startTime, r.endTime, r.ownerId);
	}
	
	map->addReservations(reservations);
}

void ReservationManager::requestPathReservation() {
	if(pathToReserve.isValid()) {
		auto_smart_factory::ReservationRequest msg;
		msg.ownerId = agentId;
		msg.bid = pathToReserve.getDuration();

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

Path ReservationManager::getReservedPath() {
	if(!hasReservedPath) {
		ROS_FATAL("[ReservationManager %d] Tried to get path but hasNoReservedPath!", agentId);
	}
	pathRetrievedCount++;
	ROS_ASSERT_MSG(pathRetrievedCount == 1, "A reserved path may only be retrieved once!!");
	
	return pathToReserve;
}

bool ReservationManager::calculateNewPath() {
	pathToReserve = map->getThetaStarPath(startPoint, endPoint, ros::Time::now().toSec() + pathStartingTimeOffset, targetReservationDuration);

	if(pathToReserve.isValid()) {
		return true;
	} else {
		ROS_FATAL("[ReservationManager %d] Tried to generate path but no valid path was found from %f/%f to %f/%f", agentId, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
		ROS_WARN("Reservations for start:");
		map->listAllReservationsIn(Point(startPoint.x, startPoint.y));

		ROS_WARN("Reservations for target:");
		map->listAllReservationsIn(Point(endPoint.x, endPoint.y));
		
		return false;
	}
}
