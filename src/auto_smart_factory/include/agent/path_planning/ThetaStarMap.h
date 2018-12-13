#ifndef PROTOTYPE_THETASTARMAP_HPP
#define PROTOTYPE_THETASTARMAP_HPP

#include <map>

#include "Math.h"
#include "agent/path_planning/GridNode.h"

#define DRAW_EDGES false

class Map;

class ThetaStarMap {
private:
	Map* map;
	std::map<Point, GridNode, Math::PointComparator> nodes;

public:
	ThetaStarMap() = default;
	ThetaStarMap(Map* map, float resolution);
	//void draw(sf::RenderWindow& renderWindow) override;

	bool isLineOfSightFree(const Point& pos1, const Point& pos2) const;
	const GridNode* getNodeClosestTo(const Point& pos) const;

private:
	void linkToNode(GridNode& node, Point targetPos);

};


#endif //PROTOTYPE_THETASTARMAP_HPP
