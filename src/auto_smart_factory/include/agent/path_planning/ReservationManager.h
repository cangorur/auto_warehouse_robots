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

	void update();
	
	// TODO add timeout
	void reservationCoordinationCallback(const auto_smart_factory::ReservationCoordination& msg);

	// Path reservations
	void bidForPathReservation(Point startPoint, Point endPoint);
	
	// Getter
	Path getReservedPath();
	bool getIsBidingForReservation() const;
	bool getHasReservedPath() const;
	
private:
	// General info
	ros::Publisher* publisher;
	Map* map;	
	int agentId;
	int agentCount;
	
	// Current auction
	int currentAuctionId;
	double currentAuctionStartTime;
	int currentAuctionReceivedBids;
	ReservationBid currentAuctionHighestBid;
	
	// Path bidding
	bool isBidingForReservation;
	bool hasReservedPath;
	Path pathToReserve;
	Point startPoint;
	Point endPoint;
	
	// Delayed auction start
	bool initializeNewDelayedAuction;
	double timestampToInitializeDelayedAuction;

	std::vector<std::pair<int, ReservationBid>> reservationBidQueue;
	
	const double emptyAuctionDelay = 0.2f;
	const double pathReservationStartingTimeOffset = 0.2f;
	
	void addReservations(const auto_smart_factory::ReservationCoordination& msg);
	void publishReservations(std::vector<Rectangle> reservations);
	void updateHighestBid(ReservationBid newBid);
	void startNewAuction(int newAuctionId, double newAuctionStartTime);
	void initializeNewAuction(int currentAuctionId, double newAuctionStartTime);
	void closeAuction();
	
};

#endif //PROJECT_RESERVATIONMANAGER_H
