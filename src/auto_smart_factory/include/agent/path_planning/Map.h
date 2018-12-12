#ifndef PROTOTYPE_MAP_H
#define PROTOTYPE_MAP_H

#include <vector>

#include "agent/path_planning/Rectangle.h"
#include "agent/path_planning/GridNode.h"
#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Path.h"

/*
#include "Dubins/OrientationPoint.hpp"
#include "Dubins/DubinsTrajectory.hpp"
#include "PathFlowField.hpp"*/

class Map {
private:
	float width;
	float height;
	float margin;
	
	std::vector<Rectangle> obstacles;
	ThetaStarMap thetaStarMap;	

public:
	Map(float width, float height, float margin, float resolutionThetaStar, std::vector<Rectangle> obstacles);

	//void draw(sf::RenderWindow& renderWindow) override;
	
	bool isInsideAnyInflatedObstacle(const Point point) const;	
	bool isLineOfSightFree(const Point& pos1, const Point& pos2) const;
	//bool isTrajectoryFree(DubinsTrajectory& trajectory) const;
	
	Point getRandomFreePoint() const;
	//OrientationPoint getRandomFreeOrientationPoint() const;
	//OrientationPoint getRandomFreeOrientationPointAlongPath(PathFlowField& pathFlowField) const;
	
	Path getThetaStarPath(const Point& start, const Point& end);
	
	// Getter
	float getWidth() const;
	float getHeight() const;
	float getMargin() const;
	
	bool isPointInMap(const Point& pos) const;

};


#endif //PROTOTYPE_MAP_H
