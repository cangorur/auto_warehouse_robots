#include <utility>
#include <iostream>
#include <include/agent/path_planning/Map.h>


#include "Math.h"
#include "agent/path_planning/Rectangle.h"
#include "agent/path_planning/Map.h"
#include "agent/path_planning/Point.h"
#include "agent/path_planning/ThetaStarPathPlanner.h"

Map::Map(float width, float height, float margin, float resolutionThetaStar, std::vector<Rectangle>& obstacles) :
		width(width),
		height(height),
		margin(margin) {
	
	this->obstacles.clear();
	for(const Rectangle& o : obstacles) {
		this->obstacles.emplace_back(o.getPosition(), o.getSize(), o.getRotation());
	}
	
	thetaStarMap = ThetaStarMap(this, resolutionThetaStar);
}

visualization_msgs::Marker Map::getVisualization() {
	visualization_msgs::Marker msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.ns = "Obstacles";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.orientation.w = 1.0;

	msg.id = 0;
	msg.type = visualization_msgs::Marker::TRIANGLE_LIST;

	msg.scale.x = 1.f;
	msg.scale.y = 1.f;
	msg.scale.z = 1.f;

	msg.color.b = 1.0f;
	msg.color.a = 0.7;

	geometry_msgs::Point p;
	p.z = 1.f;
	// Obstacles
	for(Rectangle obstacle : obstacles) {
		Point* points = obstacle.getPointsInflated();
		// First triangle
		p.x = points[0].x;
		p.y = points[0].y;
		msg.points.push_back(p);

		p.x = points[1].x;
		p.y = points[1].y;
		msg.points.push_back(p);

		p.x = points[2].x;
		p.y = points[2].y;
		msg.points.push_back(p);

		// Second triangle
		p.x = points[2].x;
		p.y = points[2].y;
		msg.points.push_back(p);

		p.x = points[3].x;
		p.y = points[3].y;
		msg.points.push_back(p);

		p.x = points[0].x;
		p.y = points[0].y;
		msg.points.push_back(p);
	}
	
	//thetaStarMap.draw(renderWindow);

	return msg;
}

bool Map::isInsideAnyInflatedObstacle(const Point& point) const {
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

Path Map::getThetaStarPath(const Point& start, const Point& end) {
	ThetaStarPathPlanner thetaStarPathPlanner(&thetaStarMap);
	return thetaStarPathPlanner.findPath(start, end);
}

bool Map::isPointInMap(const Point& pos) const {
	return pos.x >= margin && pos.x <= width - margin && pos.y >= margin && pos.y <= height - margin;
}
