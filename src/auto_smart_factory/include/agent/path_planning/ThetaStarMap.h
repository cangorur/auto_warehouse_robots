#ifndef PROTOTYPE_THETASTARMAP_HPP
#define PROTOTYPE_THETASTARMAP_HPP

#include <map>

#include "Math.h"
#include "agent/path_planning/GridNode.h"
#include "agent/path_planning/TimedLineOfSightResult.h"

#include "visualization_msgs/Marker.h"

class Map;

// Collection of Theta* Grid Nodes for Theta* Path Planning
class ThetaStarMap {
private:
	// Pointer to the normal map for line of sight checks
	Map* map;
	
	// Theta* Grid Nodes
	std::map<Point, GridNode*, Math::PointComparator> nodes;
	
	// Resolution of the Theta* Grid Nodes
	float resolution;
	
public:
	ThetaStarMap() = default;
	ThetaStarMap(Map* map, float resolution);

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
	
	/** Searches the GridNode closest to the specified position
	 * @param pos Position to search from 
	 * @return Closest grid node, nullptr if none could be found */
	const GridNode* getNodeClosestTo(const Point& pos) const;

	/** Returns a list of rectangles which contain the specified point
	 * @param p The point
	 * @return List of rectangles */
	std::vector<Rectangle> getRectanglesOnStartingPoint(Point p) const;

	/** Add a new Theta* Grid Node at the specified position and connect it to neighbouring nodes
	 * @param Pos Position for the new node
	 * @return True iff a new node was successfully added. Failure reasons include: Position is outside of the map, There already exists a GridNode at this exact position */
	bool addAdditionalNode(Point pos);
	
	/** Returns the owner if of the theta* Map
	 * @return Owner Id*/
	int getOwnerId() const;

	/** Construct RViz visualisation marker message containing the visual representation of the underlaying theta* map grid and links */
	visualization_msgs::Marker getGridVisualization();
	visualization_msgs::Marker getLinkVisualization();


private:
	/** Connect the specified Grid Node to the node at the target position
	 * @param node Node to connect 
	 * @param targetPos Position to look for the node to connect to */
	void linkToNode(GridNode* node, Point targetPos);
};


#endif //PROTOTYPE_THETASTARMAP_HPP
