#include <iostream>

#include "Math.h"
#include "agent/path_planning/Rectangle.h"
#include "agent/path_planning/Map.h"
#include "agent/path_planning/Point.h"
//#include "ThetaStar/ThetaStarPathPlanner.hpp"

Map::Map(float width, float height, float margin, float resolutionThetaStar) :
		width(width),
		height(height),
		margin(margin) {
	
	//obstacles.emplace_back(Point(0, -10), Point(8, 40), 90);
	
	//thetaStarMap = ThetaStarMap(Point(width, height), this, resolutionThetaStar);
}

/*void Map::draw(sf::RenderWindow& renderWindow) {
	// Obstacles
	for(Rectangle obstacle : obstacles) {
		obstacle.draw(renderWindow);
	}
	
	//thetaStarMap.draw(renderWindow);
}*/

bool Map::isInsideAnyInflatedObstacle(const Point point) const {
	for(const Rectangle& obstacle : obstacles) {
		if(obstacle.isInsideInflated(point)) {
			return true;
		}
	}

	return false;
}

bool Map::isLineOfSightFree(const Point& pos1, const Point& pos2) const {
	for(Rectangle obstacle : obstacles) {
		if(Math::doesLineSegmentIntersectRectangle(pos1, pos2, obstacle)) {
			return false;
		}
	}

	return true;
}

/*bool Map::isTrajectoryFree(DubinsTrajectory& trajectory) const {
	std::vector<Point>& pathPoints = trajectory.getPathPoints();
	
	int straightIndex = trajectory.getFirstStraightIndex();
	for(int i = 0; i < pathPoints.size() - 1; i++) {
		if(!isPointInMap(pathPoints[i])) {
			return false;
		}
		
		if(i == straightIndex) {
			for(Rectangle obstacle : obstacles) {
				if(Math::doesLineSegmentIntersectRectangle(pathPoints[i], pathPoints[i + 1], obstacle)) {
					return false;
				}
			}	
		} else {
			if(isInsideAnyInflatedObstacle(pathPoints[i])) {
				return false;
			}
		}		
	}

	// No need to check if last point is in map/obstacle because it was sampled valid
	return true;
}*/

Point Map::getRandomFreePoint() const {
	Point point;
	bool pointFound = false;
	
	while(!pointFound) {
		point = Point(Math::getRandom(-width/2.f, width/2.f), Math::getRandom(-height/2.f, height/2.f));
		pointFound = !isInsideAnyInflatedObstacle(point);
	}
	
	return point;	
}

/*OrientationPoint Map::getRandomFreeOrientationPoint() const {
	Point point;
	bool pointFound = false;

	while(!pointFound) {
		point = Point(Math::getRandom(-width/2.f, width/2.f), Math::getRandom(-height/2.f, height/2.f));
		pointFound = !isInsideAnyInflatedObstacle(point);
	}
	
	float angle = Math::getRandom(0, 360);

	return OrientationPoint(point, angle);
}*/

/*OrientationPoint Map::getRandomFreeOrientationPointAlongPath(PathFlowField& pathFlowField) const {
	OrientationPoint point;
	bool pointFound = false;

	while(!pointFound) {
		point = pathFlowField.getRandomPoint();
		pointFound = !isInsideAnyInflatedObstacle(point.pos) && isPointInMap(point.pos);
	}
	
	return point;
}*/

float Map::getWidth() const {
	return width;
}

float Map::getHeight() const {
	return height;
}

float Map::getMargin() const {
	return margin;
}

/*AnyAnglePath Map::getThetaStarPath(const Point& start, const Point& end) {
	ThetaStarPathPlanner thetaStarPathPlanner(&thetaStarMap);
	return thetaStarPathPlanner.findPath(start, end);
}*/

bool Map::isPointInMap(const Point& pos) const {
	return pos.x >= margin && pos.x <= width - margin && pos.y >= margin && pos.y <= height - margin;
}
