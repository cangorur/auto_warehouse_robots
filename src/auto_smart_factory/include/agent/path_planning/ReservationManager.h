#ifndef PROJECT_RESERVATIONMANAGER_H
#define PROJECT_RESERVATIONMANAGER_H

#include <ros/publisher.h>
#include <auto_smart_factory/ReservationCoordination.h>
#include "Map.h"
#include "ReservationBid.h"
#include "queue"
#include "vector"

class ReservationManager {
public:
	ReservationManager(ros::Publisher* publisher, Map* map, int agentId, int agentCount);
	~ReservationManager() = default;

	// TODO add timeout

	void reservationCoordinationCallback(const auto_smart_factory::ReservationCoordination& msg);

	// Path reservations
	void bidForPathReservation(Point startPoint, Point endPoint);
	Path getReservedPath();
	bool getIsBidingForReservation() const;
	bool getHasReservedPath() const;
	
	// TODO SEND RESERVATION MESSAGE IF WON AUCTION 
	
private:
	ros::Publisher* publisher;
	Map* map;
	
	int agentId;
	int agentCount;
	
	// Current auction
	int currentAuctionId;
	double currentAuctionStartTime;
	int currentAuctionReceivedBids;
	ReservationBid currentAuctionHighestBid;
	
	bool isBidingForReservation;
	bool hasReservedPath;
	Path pathToReserve;
	
	void addReservations(const auto_smart_factory::ReservationCoordination& msg);
	void updateBid(ReservationBid newBid);
	void startNewAuction(int newAuctionId, double newAuctionStartTime);
	void closeAuction();
	
};

#endif //PROJECT_RESERVATIONMANAGER_H
