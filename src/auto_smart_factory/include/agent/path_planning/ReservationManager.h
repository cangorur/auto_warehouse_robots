#ifndef PROJECT_RESERVATIONMANAGER_H
#define PROJECT_RESERVATIONMANAGER_H

#include "ros/publisher.h"
#include "auto_smart_factory/ReservationBroadcast.h"
#include "agent/path_planning/OrientedPoint.h"
#include "Map.h"
#include "queue"
#include "vector"

/* Class which takes care of path reservations and handles the coordination communication for the agent */
class ReservationManager {
public:
	/** Constructor
	 * @param publisher Publisher for the coordination topic 
	 * @param Map The map 
	 * @param agentID The agent id of the robots this reservations manager belongs to
	 * @param warehouseConfig The warehouse configuration containing the starting positions*/
	ReservationManager(ros::Publisher* publisher, Map* map, int agentId, auto_smart_factory::WarehouseConfiguration warehouseConfig);
	~ReservationManager() = default;

	/** Update function which checks if replanning is necessary and deletes expires reservations
	 * @param pos Current robot position */
	void update(Point pos);

	/** Callback for the reservations coordination communication
	 * @param msg The received message */
	void reservationBroadcastCallback(const auto_smart_factory::ReservationBroadcast& msg);

	/** Start to bid for a path reservation
	 * @param startPoint Path start point
	 * @param endPoint Path end point
	 * @param targetReservationDuration Reservation duration at path end point */
	void startBiddingForPathReservation(OrientedPoint startPoint, OrientedPoint endPoint, double targetReservationDuration);
	
	/** Publishes an emergency stop message at the specified position 
	 * @param pos Emergency stop position */
	void publishEmergencyStop(Point pos);
	
	// Getter
	Path getLastReservedPath();
	bool isBidingForReservation() const;
	bool getHasReservedPath() const;
	bool hasRequestedEmergencyStop() const;
	bool isReplanningNecessary() const;
	bool isReplanningBeneficial() const;
	
private:
	// Communication publisher
	ros::Publisher* publisher;
	
	// Pointer to Map
	Map* map;
	
	// Agent id of the owning agent
	int agentId;
	
	// Is this robot currently bidding for a path
	bool bidingForReservation;
	
	// Has this reservation Manager a reserved path
	bool hasReservedPath;
	
	// The current path to reserve
	Path pathToReserve;
	
	// Current path start/end point
	OrientedPoint startPoint;
	OrientedPoint endPoint;
	
	// Current path target reservation duration
	double targetReservationDuration;
	
	// The times this path has been retrieved
	int pathRetrievedCount;
	
	/** Save the reservations in the message as the last reserved path reservations. These are used to check if the agent is currently inside one of its own reservations 
	 * @param msg Message contaiing the last reserved reservations */
	void saveReservationsAsLastReserved(const auto_smart_factory::ReservationBroadcast& msg);
	
	/** Extracts the reservations for a reservation broadcast message
	 * @param msg The message contaiing the reservations */
	std::vector<Rectangle> getReservationsFromMessage(const auto_smart_factory::ReservationBroadcast& msg);
	
	/** Request reservations for the current path */
	void requestPathReservation();
	
	/** Calculate new Path from current start to finish */
	bool calculateNewPath();
	
	// Last reserved reservations. Used to check if the robot is currently in a spot reserved by him
	std::vector<Rectangle> lastReservedPathReservations;
	
	// Is replanning necessary
	bool replanningNecessary;
	
	// Has this Reservation Manager requested an emergency stop
	bool requestedEmergencyStop;
	
	/** Checks if the robot currently is in one of his own reservations
	 * @param pos Current robot position 
	 * @param time Current time 
	 * @return Return true iff the robot currently is in hiw own reservation, position and time wise */
	bool isInOwnReservation(Point pos, double time);
	
	/** Checks if a new emergency stop lies on the path in front of the robot 
	 * @param emergencyStopReservations Reservations of the emergency stop 
	 * @return True iff the emergency stop blocks the robots path and therefore replanning is necessary */ 
	bool doesEmergencyStopPreventsOwnPath(const std::vector<Rectangle>& emergencyStopReservations) const;
	
	/** Checks if replanning would be beneficial if these old reservations are gone 
	 * @param oldReservations Reservations which where deleted recently 
	 * @return True iff a better path to the current target exists because the old reservations are gone */
	bool isReplanningBeneficialWithoutTheseReservations(const std::vector<Rectangle>& oldReservations) const;
	
	// Is replanning beneficial eg. does a more optimal path exist
	bool replanningBeneficial;	
};

#endif //PROJECT_RESERVATIONMANAGER_H
