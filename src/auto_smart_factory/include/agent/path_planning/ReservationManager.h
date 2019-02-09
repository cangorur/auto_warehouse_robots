#ifndef PROJECT_RESERVATIONMANAGER_H
#define PROJECT_RESERVATIONMANAGER_H

#include "ros/publisher.h"
#include "auto_smart_factory/ReservationBroadcast.h"
#include "agent/path_planning/OrientedPoint.h"
#include "Map.h"
#include "queue"
#include "vector"

class ReservationManager {
public:
	ReservationManager(ros::Publisher* publisher, Map* map, int agentId);
	~ReservationManager() = default;

	void update();
	void reservationBroadcastCallback(const auto_smart_factory::ReservationBroadcast& msg);

	// Path reservations
	void startBiddingForPathReservation(OrientedPoint startPoint, OrientedPoint endPoint, double targetReservationDuration);
	
	// Getter
	Path getReservedPath();
	bool isBidingForReservation() const;
	bool getHasReservedPath() const;
	
private:
	// General info
	ros::Publisher* publisher;
	Map* map;	
	int agentId;
	
	// Path bidding
	bool bidingForReservation;
	bool hasReservedPath;
	Path pathToReserve;
	OrientedPoint startPoint;
	OrientedPoint endPoint;
	double targetReservationDuration;
	int pathRetrievedCount;
	
	const double pathStartingTimeOffset = 0.0f;
	
	void addReservations(const auto_smart_factory::ReservationBroadcast& msg);
	void requestPathReservation();
	bool calculateNewPath();
	
};

#endif //PROJECT_RESERVATIONMANAGER_H
