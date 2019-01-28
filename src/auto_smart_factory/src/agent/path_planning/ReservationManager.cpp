
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
	
	currentAuctionId = 0;

	if(agentId == 1) {
		initializeNewDelayedAuction = true;
		timestampToInitializeDelayedAuction = ros::Time::now().toSec() + ros::Duration(0.1f).toSec();
	} else {
		initializeNewDelayedAuction = false;
	}
}

void ReservationManager::update() {
	map->deleteExpiredReservations(ros::Time::now().toSec());
	
	// Start new delayed auction
	if(initializeNewDelayedAuction && ros::Time::now().toSec() >= timestampToInitializeDelayedAuction) {
		initializeNewDelayedAuction = false;
		double startTime = ros::Time::now().toSec();
		
		initializeNewAuction(currentAuctionId, startTime);		
		startNewAuction(currentAuctionId + 1, startTime);
	}
	
	// Process queued reservationBids
	auto iter = reservationBidQueue.begin();

	while(iter != reservationBidQueue.end()) {
		if((*iter).first < currentAuctionId) {
			iter = reservationBidQueue.erase(iter);
		} else if((*iter).first == currentAuctionId) {
			updateBid(ReservationBid((*iter).second.bid, (*iter).second.agentId));
			
			iter = reservationBidQueue.erase(iter);
		} else {
			iter++;
		}
	}
}

void ReservationManager::reservationCoordinationCallback(const auto_smart_factory::ReservationCoordination& msg) {
	// Ignore own messages
	if(msg.robotId == agentId) {
		return;
	}
	
	if(msg.auctionId != currentAuctionId) {
		// Queue out of order bidding messages, process reservation messages
		if(!msg.isReservationMessage) {
			ROS_ERROR("[Agent %d] Got bid with auction id %d but current auctionId is %d", agentId, msg.auctionId, currentAuctionId);
			
			if(msg.auctionId > currentAuctionId) {
				reservationBidQueue.emplace_back(msg.auctionId, ReservationBid(msg.bid, msg.robotId));
			}
						
			return;
		}
	}
	
	if(msg.isReservationMessage) {
		addReservations(msg);
		startNewAuction(msg.auctionId + 1, msg.timestamp);
	} else {
		updateBid(ReservationBid(msg.bid, msg.robotId));
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

	if(currentAuctionReceivedBids >= agentCount) {
		closeAuction();
	}
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

	// Update own bid
	updateBid(ReservationBid(msg.bid, agentId));
	
	publisher->publish(msg);	
}

void ReservationManager::closeAuction() {
	if(currentAuctionHighestBid.agentId == agentId) {
		ROS_INFO("[ReservationManager %d] Won auction for path with duration %f", agentId, pathToReserve.getDuration());
		isBidingForReservation = false;
		hasReservedPath = true;

		std::vector<Rectangle> reservations = pathToReserve.generateReservations(agentId);
		map->addReservations(reservations);

		publishReservations(reservations);		
		startNewAuction(currentAuctionId + 1, ros::Time::now().toSec());
	} else {
		if(isBidingForReservation) {
			// Recalculate path to match timing
			pathToReserve = map->getThetaStarPath(pathToReserve.getNodes().front(), pathToReserve.getNodes().back(), ros::Time::now().toSec());

		} else if(std::abs(currentAuctionHighestBid.bid - -1.f) <= EPS) {
			// If no one is bidding at all => no winner message => no new auction
			if(currentAuctionHighestBid.agentId == -1) {
				ROS_ERROR("Auction closes but no bid with agentId != 0 found!");
			} else {
				// Agent with highest id starts new auction but delayed to avoid overhead
				if(currentAuctionHighestBid.agentId == agentId) {
					initializeNewDelayedAuction = true;
					timestampToInitializeDelayedAuction = ros::Time::now().toSec() + ros::Duration(0.1f).toSec();
				}	
			}
		}		
	}
}

void ReservationManager::publishReservations(std::vector<Rectangle> reservations) {
	auto_smart_factory::ReservationCoordination msg;
	msg.timestamp = ros::Time::now().toSec();
	msg.robotId = agentId;
	msg.auctionId = currentAuctionId;
	msg.isReservationMessage = 1;
	
	for(const auto& r : reservations) {
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

void ReservationManager::initializeNewAuction(int newAuctionId, double newAuctionStartTime) {
	auto_smart_factory::ReservationCoordination msg;
	msg.timestamp = newAuctionStartTime;
	msg.robotId = agentId;
	msg.auctionId = newAuctionId;
	msg.isReservationMessage = 1;
	msg.reservations.clear();

	publisher->publish(msg);
}
