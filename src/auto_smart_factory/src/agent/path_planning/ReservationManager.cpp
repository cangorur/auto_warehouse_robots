
#include "agent/path_planning/ReservationManager.h"

ReservationManager::ReservationManager(ros::Publisher* publisher, Map* map, int agentId, int agentCount) :
	publisher(publisher),
	map(map),
	agentId(agentId),
	agentCount(agentCount),
	currentAuctionId(0),
	currentAuctionHighestBid(-1.f, -1)
{
	hasReservedPath = false;
	bidingForReservation = false;

	if(agentId == agentCount) {
		initializeNewDelayedAuction = true;
		timestampToInitializeDelayedAuction = ros::Time::now().toSec() + emptyAuctionDelay;
	} else {
		initializeNewDelayedAuction = false;
	}
}

void ReservationManager::update() {
	double now = ros::Time::now().toSec();
	map->deleteExpiredReservations(now);
	
	// Start new delayed auction
	if(initializeNewDelayedAuction && now >= timestampToInitializeDelayedAuction) {
		initializeNewDelayedAuction = false;

		int auctionId = currentAuctionId;
		startNewAuction(auctionId + 1, now);
		initializeNewAuction(auctionId, now);		
	}
	
	// Process queued reservationBids
	auto iter = reservationBidQueue.begin();
	while(iter != reservationBidQueue.end()) {
		if((*iter).first < currentAuctionId) {
			iter = reservationBidQueue.erase(iter);
		} else if((*iter).first == currentAuctionId) {
			updateHighestBid(ReservationBid((*iter).second.bid, (*iter).second.agentId));
			iter = reservationBidQueue.erase(iter);
		} else {
			iter++;
		}
	}
	
	// Auction timeout
	if(now >= currentAuctionStartTime + auctionTimeout && currentAuctionId > 1) {
		if(agentId == currentAuctionHighestBid.agentId) {
			ROS_ERROR("[ReservationManager %d]: Auction %d timed out. %d agents left", agentId, currentAuctionId, currentAuctionReceivedBids);
		}
		agentCount = currentAuctionReceivedBids;
		startNewAuction(currentAuctionId + 1, now);
	}	
}

void ReservationManager::reservationCoordinationCallback(const auto_smart_factory::ReservationCoordination& msg) {
	// Ignore own messages
	if(msg.robotId == agentId) {
		return;
	}

	// Queue out of order bidding messages, process reservation messages
	if(msg.auctionId != currentAuctionId && !msg.isReservationMessage) {
		if(msg.auctionId > currentAuctionId) {
			reservationBidQueue.emplace_back(msg.auctionId, ReservationBid(msg.bid, msg.robotId));
		}
					
		return;
	}
	
	if(msg.isReservationMessage) {
		map->deleteReservationsFromAgent(msg.robotId);
		addReservations(msg);
		startNewAuction(msg.auctionId + 1, msg.timestamp);
	} else {
		updateHighestBid(ReservationBid(msg.bid, msg.robotId));
	}
}

void ReservationManager::addReservations(const auto_smart_factory::ReservationCoordination& msg) {
	std::vector<Rectangle> reservations;
	
	for(auto r : msg.reservations) {
		reservations.emplace_back(Point(r.posX, r.posY), Point(r.sizeX, r.sizeY), r.rotation, r.startTime, r.endTime, r.ownerId);
	}
	
	map->addReservations(reservations);
}

void ReservationManager::updateHighestBid(ReservationBid newBid) {
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
	msg.isReservationMessage = static_cast<unsigned char>(false);

	if(bidingForReservation) {
		// Recalculate path to match timing
		pathToReserve = map->getThetaStarPath(startPoint, endPoint, ros::Time::now().toSec() + pathReservationStartingTimeOffset, targetReservationDuration);

		if(!pathToReserve.isValid()) {
			ROS_ERROR("[ReservationManager %d] Tried to re-generate path but no valid path was found from %f/%f to %f/%f", agentId, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
		}
	}

	if(bidingForReservation) {
		//ROS_INFO("[ReservationManager %d] Starting new auction %d - bidding", agentId, newAuctionId);
		msg.bid = static_cast<float>(pathToReserve.getDuration());
	} else {
		msg.bid = -1.f;
	}

	// Update own bid
	updateHighestBid(ReservationBid(msg.bid, agentId));
	
	publisher->publish(msg);	
}

void ReservationManager::closeAuction() {
	if(currentAuctionHighestBid.agentId == agentId) {
		if(bidingForReservation) {
			ROS_INFO("[ReservationManager %d] Won auction %d for path with duration %f", agentId, currentAuctionId, pathToReserve.getDuration());
			bidingForReservation = false;
			hasReservedPath = true;
			pathRetrievedCount = 0;

			// Delete old reservations and add new
			map->deleteReservationsFromAgent(agentId);
			std::vector<Rectangle> reservations = pathToReserve.generateReservations(agentId);
			map->addReservations(reservations);

			publishReservations(reservations);
			startNewAuction(currentAuctionId + 1, ros::Time::now().toSec());	
		} else {
			//ROS_INFO("[ReservationManager %d] Won empty auction %d initializing new auction", agentId, currentAuctionId);

			// Agent with highest id starts new auction but delayed to avoid overhead
			initializeNewDelayedAuction = true;
			timestampToInitializeDelayedAuction = ros::Time::now().toSec() + emptyAuctionDelay;
		}	
	} 
}

void ReservationManager::publishReservations(std::vector<Rectangle> reservations) {
	auto_smart_factory::ReservationCoordination msg;
	msg.timestamp = ros::Time::now().toSec();
	msg.robotId = agentId;
	msg.auctionId = currentAuctionId;
	msg.isReservationMessage = static_cast<unsigned char>(true);
	
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

void ReservationManager::startBiddingForPathReservation(OrientedPoint startPoint, OrientedPoint endPoint, double targetReservationDuration) {
	// Use Radiant here	
	this->startPoint = startPoint;
	this->endPoint = endPoint;	
	this->targetReservationDuration = targetReservationDuration;
	
	pathToReserve = map->getThetaStarPath(startPoint, endPoint, ros::Time::now().toSec() + pathReservationStartingTimeOffset, targetReservationDuration);
	if(pathToReserve.isValid()) {
		bidingForReservation = true;
		hasReservedPath = false;

	} else {
		ROS_ERROR("[ReservationManager %d] Tried to generate path but no valid path was found from %f/%f to %f/%f", agentId, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
		hasReservedPath = false;
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

void ReservationManager::initializeNewAuction(int auctionId, double newAuctionStartTime) {
	auto_smart_factory::ReservationCoordination msg;
	msg.timestamp = newAuctionStartTime;
	msg.robotId = agentId;
	msg.auctionId = auctionId;
	msg.isReservationMessage = static_cast<unsigned char>(true);
	msg.reservations.clear();

	publisher->publish(msg);
}
