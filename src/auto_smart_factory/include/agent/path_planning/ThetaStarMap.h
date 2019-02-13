#ifndef PROTOTYPE_THETASTARMAP_HPP
#define PROTOTYPE_THETASTARMAP_HPP

#include <map>

#include "Math.h"
#include "agent/path_planning/GridNode.h"
#include "agent/path_planning/TimedLineOfSightResult.h"

class Map;

class ThetaStarMap {
private:
	Map* map;
	std::map<Point, GridNode*, Math::PointComparator> nodes;
	float resolution;
	
public:
	ThetaStarMap() = default;
	ThetaStarMap(Map* map, float resolution);

	TimedLineOfSightResult whenIsTimedLineOfSightFree(const Point& pos1, double startTime, const Point& pos2, double endTime, std::vector<Rectangle>& reservationsToIgnore) const;
	bool isTimedConnectionFree(const Point& pos1, const Point& pos2, double startTime, double waitingTime, double drivingTime, std::vector<Rectangle>& reservationsToIgnore) const;
	
	const GridNode* getNodeClosestTo(const Point& pos) const;

	std::vector<Rectangle> getRectanglesOnStartingPoint(Point p) const;

	bool addAdditionalNode(Point pos);
	
	// Debugging
	void listAllReservationsIn(Point p);
	int getOwnerId() const;

private:
	void linkToNode(GridNode* node, Point targetPos);

};


#endif //PROTOTYPE_THETASTARMAP_HPP
