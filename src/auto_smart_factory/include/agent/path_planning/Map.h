#ifndef PROTOTYPE_MAP_H
#define PROTOTYPE_MAP_H

#include <vector>

#include "auto_smart_factory/Tray.h"
#include "auto_smart_factory/WarehouseConfiguration.h"
#include "agent/path_planning/Rectangle.h"
#include "agent/path_planning/GridNode.h"
#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Path.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/RobotHardwareProfile.h"
#include "agent/path_planning/TimedLineOfSightResult.h"

/* Represents the agents local copy of the map. Contains static obstacles, timed reservations and a theta star map (collection of theta star grid nodes) */
class Map {
public:
	/* Duration to use for a infinite reservation */
	static double infiniteReservationTime;
	
private:
	// For visualisation messages
	static int visualisationId;
	
	/* Map measurements and warehouse config */
	float width;
	float height;
	float margin;
	auto_smart_factory::WarehouseConfiguration warehouseConfig;
	
	// Static obstacles, set in constructor
	std::vector<Rectangle> obstacles;
	
	// Timed reservations, including own reservations
	std::vector<Rectangle> reservations;
	
	// Theta star map used for theta star path queries
	ThetaStarMap thetaStarMap;
	
	// Hardware profile of the agent using this map. Necessary for path estimations (battery, duration...)
	RobotHardwareProfile* hardwareProfile;
	
	// Id of the owning agent. Used to determine own reservations (which should be ignored)
	int ownerId;

public:
	Map(auto_smart_factory::WarehouseConfiguration warehouseConfig, std::vector<Rectangle> &obstacles, RobotHardwareProfile* hardwareProfile, int ownerId);
	~Map() = default;

	/** Construct RViz visualisation marker message containing the visual representation of the obstacles */
	visualization_msgs::Marker getObstacleVisualization();

	/** Construct RViz visualisation marker message containing the visual representation of the current inactive reservations
	 * @param baseColor color the reservations shall have in RVIZ */
	visualization_msgs::Marker getInactiveReservationVisualization(visualization_msgs::Marker::_color_type baseColor);

	/** Construct RViz visualisation marker message containing the visual representation of the current active reservations
	 * @param baseColor color the reservations shall have in RVIZ */
	visualization_msgs::Marker getActiveReservationVisualization(visualization_msgs::Marker::_color_type baseColor);

	/** Checks whether a point is inside any static obstacle
	 * @param point the point to check
	 * @return true/false if the point is inside any static obstacle */
	bool isInsideAnyStaticInflatedObstacle(const Point& point) const;

	/** Checks whether a the line of sight between 2 points is free
	 * @param pos1 Start point
	 * @param pos2 End point
	 * @return true/false if the point line of sight is free */
	bool isStaticLineOfSightFree(const Point& pos1, const Point& pos2) const;

	/** Compute if and when a certain line of sight connection is free
	 * @param pos1 Start point
	 * @param pos2 End point
	 * @param startTime time when the connection starts at the start point
	 * @param endTime time when the connection is planned to end at the end point
	 * @param smallerReservations list of reservations where a smaller variant should be used because the robot starts in these reservations
	 * @return TimedLineOfSighResult. Check @class TimedLineOfSightResult for more info*/ 
	TimedLineOfSightResult whenIsTimedLineOfSightFree(const Point& pos1, double startTime, const Point& pos2, double endTime, const std::vector<Rectangle>& smallerReservations) const;

	/** Check if certain line of sight connection is actually free for both driving and waiting during the connection
	 * @param pos1 Start point
	 * @param pos2 End point
	 * @param startTime time when the connection starts at the start point
	 * @param waitingTime Time to wait at the start point
	 * @param drivingTime Time to drive from start to end
	 * @param smallerReservations list of reservations where a smaller variant should be used because the robot starts in these reservations
	 * @return TimedLineOfSighResult. Check @class TimedLineOfSightResult for more info*/
	bool isTimedConnectionFree(const Point& pos1, const Point& pos2, double startTime, double waitingTime, double drivingTime, const std::vector<Rectangle>& smallerReservations) const;
	
	/** Checks whether a certain point is in the map 
	 * @param pos the point to check
	 * @return true iff point in map */
	bool isPointInMap(const Point& pos) const;

	/** Computes the approach point in front of the specified tray
	 * @param tray The tray
	 * @return The oriented point in front of the tray */
	OrientedPoint getPointInFrontOfTray(const auto_smart_factory::Tray& tray);

	/** Adds all reservations to the map
	 * @param newReservations list of reservations to add */
	void addReservations(const std::vector<Rectangle>& newReservations);
	
	/** Delete all reservations which are expired at this time
	 * @param time the current time */
	void deleteExpiredReservations(double time);
	
	/** Delete all reservations from this agent 
	 * @param the agent id from which to delete reservations */
	std::vector<Rectangle> deleteReservationsFromAgent(int agentId);
		
	/** Compute a theta star path inside this map. If a OrientedPoint is given, use this point, if a tray is given, compute the approach point in front of this tray and use this point instead
	 * @param start start point or tray for path
	 * @param end end point or tray for the path
	 * @param startingTime The time point when the path should start
	 * @param targetReservationTime Duration the reservations at the end of the path should last
	 * @param ignoreStartingReservations Ignore any reservations the start point is inside 
	 * @return The computed Path. Check path.isValid before using it, errors are returned via an invalid path object */
	Path getThetaStarPath(const OrientedPoint& start, const OrientedPoint& end, double startingTime, double targetReservationTime, bool ignoreStartingReservations);
	Path getThetaStarPath(const OrientedPoint& start, const auto_smart_factory::Tray& end, double startingTime, double targetReservationTime);
	Path getThetaStarPath(const auto_smart_factory::Tray& start, const OrientedPoint& end, double startingTime, double targetReservationTime);
	Path getThetaStarPath(const auto_smart_factory::Tray& start, const auto_smart_factory::Tray& end, double startingTime, double targetReservationTime);
	
	/** Checks whether a position is the current target of another robot 
	 * @param pos The position to check 
	 * @return True iff the position is the current target of any other robot */
	bool isPointTargetOfAnotherRobot(OrientedPoint pos);

	/** Checks whether the closest GridNodes to two points are neighbors in the ThetaStarMap
	 * @param pos1 The position of the first node
	 * @param pos2 The position of the second node
	 * @return True or False */
	bool arePointsConnected(const Point& pos1, const Point& pos2) const;
	
	// Getter
	float getWidth() const;
	float getHeight() const;
	float getMargin() const;
	int getOwnerId() const;
	bool getLineFollowingFlag() const;

	std::vector<Rectangle> getRectanglesOnStartingPoint(Point p) const;

private:

};


#endif //PROTOTYPE_MAP_H
