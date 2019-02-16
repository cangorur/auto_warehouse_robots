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
	ReservationManager(ros::Publisher* publisher, Map* map, int agentId, Point startingPosition);
	~ReservationManager() = default;

	void update(Point pos);
	void reservationBroadcastCallback(const auto_smart_factory::ReservationBroadcast& msg);

	// Path reservations
	void startBiddingForPathReservation(OrientedPoint startPoint, OrientedPoint endPoint, double targetReservationDuration);
	void publishEmergencyStop(Point pos);
	
	// Getter
	Path getLastReservedPath();
	bool isBidingForReservation() const;
	bool getHasReservedPath() const;
	OrientedPoint getLastReservedPathTarget() const;
	bool hasRequestedEmergencyStop() const;
	bool isReplanningNecessary() const;
	
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
	
	void saveReservationsAsLastReserved(const auto_smart_factory::ReservationBroadcast& msg);
	std::vector<Rectangle> getReservationsFromMessage(const auto_smart_factory::ReservationBroadcast& msg);
	void requestPathReservation();
	bool calculateNewPath();
	
	// Error State detection
	OrientedPoint lastReservedPathTarget;
	std::vector<Rectangle> lastReservedPathReservations;
	bool replanningNecessary;
	bool requestedEmergencyStop;
	
	bool isInOwnReservation(Point pos, double time);
	bool doesEmergencyStopPreventsOwnPath(const std::vector<Rectangle>& emergencyStopReservations) const;
	
};

#endif //PROJECT_RESERVATIONMANAGER_H
