
#include <include/agent/path_planning/ReservationManager.h>

#include "agent/path_planning/ReservationManager.h"

ReservationManager::ReservationManager(ros::Publisher* publisher, Map* map, int agentId, int agentCount) :
	publisher(publisher),
	map(map),
	agentId(agentId),
	agentCount(agentCount)
{
	hasReservedPath = false;
	isBidingForReservation = false;
	
	// Todo init CurrentAuctionID
}

void ReservationManager::reservationCoordinationCallback(const auto_smart_factory::ReservationCoordination& msg) {
	// ToDo Race Conditions: IF getting msg for auction id + 1 queue them and add them to auction later
	if(msg.auctionId != currentAuctionId) {
		ROS_ERROR("[Agent %d] Got bid with auction id %d but current auctionId is %d", agentId, msg.auctionId, currentAuctionId);
		return;
	}
	
	if(msg.isReservationMessage) {
		addReservations(msg);
		startNewAuction(msg.auctionId + 1, msg.timestamp);
	} else {
		updateBid(ReservationBid(msg.bid, msg.robotId));
		
		if(currentAuctionReceivedBids >= agentCount) {
			closeAuction();
		}
	}
}

void ReservationManager::addReservations(const auto_smart_factory::ReservationCoordination& msg) {
	std::vector<Rectangle> reservations;
	
	for(auto r : msg.reservations) {
		reservations.emplace_back(Point(r.posX, r.posY), Point(r.sizeX, r.sizeY), r.rotation, r.startTime, r.endTime, r.ownerId);
	}
	
	map->addReservations(reservations);
}

void ReservationManager::updateBid(ReservationBid newBid) {
	if(newBid.bid > currentAuctionHighestBid.bid || (newBid.bid == currentAuctionHighestBid.bid && newBid.agentId > currentAuctionHighestBid.agentId)) {
		currentAuctionHighestBid = newBid;
	}
	
	currentAuctionReceivedBids++;
}

void ReservationManager::startNewAuction(int newAuctionId, double newAuctionStartTime) {
	currentAuctionId = newAuctionId;
	currentAuctionStartTime = newAuctionStartTime;
	currentAuctionHighestBid = ReservationBid(-1.f, -1);
	currentAuctionReceivedBids = 0;
	
	// Send current bid if bidding, send emtpy bid otherwise
	auto_smart_factory::ReservationCoordination msg;
	msg.timestamp = ros::Time::now().toSec();
	msg.robotId = agentId;
	msg.auctionId = currentAuctionId;
	msg.isReservationMessage = 0;
	
	if(isBidingForReservation) {
		msg.bid = static_cast<float>(pathToReserve.getDuration());
	} else {
		msg.bid = -1.f;
	}
	
	publisher->publish(msg);
}

void ReservationManager::closeAuction() {
	if(currentAuctionHighestBid.agentId == agentId) {
		// Reserve path
		map->addReservations(pathToReserve.generateReservations());
		
		isBidingForReservation = false;
		hasReservedPath = true;

		ROS_INFO("[ReservationManager %d] Won auction for path with duration %f", agentId, pathToReserve.getDuration());
	} else {
		if(isBidingForReservation) {
			// Recalculate path to match timing
			pathToReserve = map->getThetaStarPath(pathToReserve.getNodes().front(), pathToReserve.getNodes().back(), ros::Time::now().toSec());
		} else {
			// Check if anyone is biding at all
			if(currentAuctionHighestBid.agentId == -1) {
				// TODO solve this. Either start immideately -> overhead OR delay new Auction OR go to NO_Bidder state and the agent first who wants to bid starts a new auction (what if two start at the same time...)
				startNewAuction(currentAuctionId + 1, ros::Time::now().toSec());
			}
		}		
	}
}

void ReservationManager::bidForPathReservation(Point startPoint, Point endPoint) {
	isBidingForReservation = true;
	hasReservedPath = false;

	pathToReserve = map->getThetaStarPath(startPoint, endPoint, ros::Time::now().toSec());

	ROS_INFO("[ReservationManager %d] Starting to bid for path with duration %f", agentId, pathToReserve.getDuration());
}

bool ReservationManager::getIsBidingForReservation() const {
	return isBidingForReservation;
}

bool ReservationManager::getHasReservedPath() const {
	return hasReservedPath;
}

Path ReservationManager::getReservedPath() {
	return pathToReserve;
}
