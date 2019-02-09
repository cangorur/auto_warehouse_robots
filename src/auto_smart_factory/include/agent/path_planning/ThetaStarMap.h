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

	bool isStaticLineOfSightFree(const Point& pos1, const Point& pos2) const;
	bool isTimedLineOfSightFree(const Point& pos1, double startTime, const Point& pos2, double endTime) const;
	TimedLineOfSightResult whenIsTimedLineOfSightFree(const Point& pos1, double startTime, const Point& pos2, double endTime) const;
	bool isTimedConnectionFree(const Point& pos1, const Point& pos2, double startTime, double waitingTime, double drivingTime) const;
	const GridNode* getNodeClosestTo(const Point& pos) const;

	const GridNode* addAdditionalNode(Point pos);
	
	// Debugging
	void listAllReservationsIn(Point p);
	int getOwnerId() const;

private:
	void linkToNode(GridNode* node, Point targetPos);

};


#endif //PROTOTYPE_THETASTARMAP_HPP
