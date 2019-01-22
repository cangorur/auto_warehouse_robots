#include <utility>
#include <iostream>
#include <include/agent/path_planning/Map.h>

#include "Math.h"
#include "agent/path_planning/Rectangle.h"
#include "agent/path_planning/Map.h"
#include "agent/path_planning/Point.h"
#include "agent/path_planning/ThetaStarPathPlanner.h"

Map::Map(auto_smart_factory::WarehouseConfiguration warehouseConfig, std::vector<Rectangle>& obstacles) :
		warehouseConfig(warehouseConfig),
		width(warehouseConfig.map_configuration.width),
		height(warehouseConfig.map_configuration.height),
		margin(warehouseConfig.map_configuration.margin) {
	
	this->obstacles.clear();
	for(const Rectangle& o : obstacles) {
		this->obstacles.emplace_back(o.getPosition(), o.getSize(), o.getRotation());
	}
	
	thetaStarMap = ThetaStarMap(this, warehouseConfig.map_configuration.resolutionThetaStar);
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
		const Point* points = obstacle.getPointsInflated();
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

bool Map::isStaticLineOfSightFree(const Point& pos1, const Point& pos2) const {
	for(Rectangle obstacle : obstacles) {
		if(Math::doesLineSegmentIntersectRectangle(pos1, pos2, obstacle)) {
			return false;
		}
	}

	return true;
}

bool Map::isTimedLineOfSightFree(const Point& pos1, float startTime, const Point& pos2, float endTime) const {
	if(!isStaticLineOfSightFree(pos1, pos2)) {
		return false;
	}

	for(const Rectangle& reservation : reservations) {
		if(reservation.doesOverlapTimeRange(startTime, endTime) && Math::doesLineSegmentIntersectRectangle(pos1, pos2, reservation)) {
			return false;
		}
	}

	return true;
}

TimedLineOfSightResult Map::whenIsTimedLineOfSightFree(const Point& pos1, float startTime, const Point& pos2, float endTime) const {
	TimedLineOfSightResult result;

	if(!isStaticLineOfSightFree(pos1, pos2)) {
		result.blockedByStatic = true;
		return result;
	}

	for(const Rectangle& reservation : reservations) {
		// Directly blocked
		if(reservation.doesOverlapTimeRange(startTime + 0.01f, endTime) && Math::doesLineSegmentIntersectRectangle(pos1, pos2, reservation)) {
			result.blockedByTimed = true;
			if(reservation.getFreeAfter() > result.freeAfter) {
				result.freeAfter = reservation.getFreeAfter();
			}
		}

		// Upcoming obstacles			
		if(Math::isPointInRectangle(pos2, reservation) && reservation.getStartTime() > endTime) {
			result.hasUpcomingObstacle = true;
			// Todo make adaptive
			float minTimeToLeave = 16;

			float lastValidEntryTime = reservation.getStartTime() - minTimeToLeave;
			if(lastValidEntryTime < result.lastValidEntryTime) {
				result.lastValidEntryTime = lastValidEntryTime;
				result.freeAfterUpcomingObstacle = reservation.getEndTime();
			}
		}
	}

	return result;
}

bool Map::isTimedConnectionFree(const Point& pos1, const Point& pos2, float startTime, float waitingTime, float drivingTime) const {
	float endTime = startTime + waitingTime + drivingTime;

	for(const Rectangle& reservation : reservations) {
		// Check if the waiting part is free
		if(reservation.doesOverlapTimeRange(startTime, startTime + waitingTime) && Math::isPointInRectangle(pos1, reservation)) {
			//printf("Connection dropped due to waiting: %.1f/%.1f -> %.1f/%.1f : wait: %.1f, drive: %.1f\n", pos1.x, pos1.y, pos2.x, pos2.y, waitingTime, drivingTime);
			return false;
		}

		// Check if the driving part is free
		if(reservation.doesOverlapTimeRange(startTime + waitingTime + 0.001f, endTime) && Math::doesLineSegmentIntersectRectangle(pos1, pos2, reservation)) {
			//printf("Connection dropped due to driving: %.1f/%.1f -> %.1f/%.1f : wait: %.1f, drive: %.1f\n", pos1.x, pos1.y, pos2.x, pos2.y, waitingTime, drivingTime);
			return false;
		}
	}

	return true;
}

Point Map::getRandomFreePoint() const {
	Point point;
	bool pointFound = false;
	
	while(!pointFound) {
		point = Point(Math::getRandom(-width/2.f, width/2.f), Math::getRandom(-height/2.f, height/2.f));
		pointFound = !isInsideAnyInflatedObstacle(point);
	}
	
	return point;	
}

float Map::getWidth() const {
	return width;
}

float Map::getHeight() const {
	return height;
}

float Map::getMargin() const {
	return margin;
}

bool Map::isPointInMap(const Point& pos) const {
	return pos.x >= margin && pos.x <= width - margin && pos.y >= margin && pos.y <= height - margin;
}

void Map::deleteExpiredReservations(float time) {
	auto iter = reservations.begin();

	while(iter != reservations.end()) {
		if((*iter).getEndTime() > time + 0.5f) {
			iter = reservations.erase(iter);
		} else {
			iter++;
		}
	}
}

void Map::addReservations(std::vector<Rectangle> newReservations) {
	for(const auto& r : newReservations) {
		reservations.emplace_back(r.getPosition(), r.getSize(), r.getRotation(), r.getStartTime(), r.getEndTime());
	}
}

Path Map::getThetaStarPath(const Point& start, const Point& end, float startingTime, RobotHardwareProfile* hardwareProfile) {
	ThetaStarPathPlanner thetaStarPathPlanner(&thetaStarMap, hardwareProfile);
	return thetaStarPathPlanner.findPath(start, end, startingTime);
}
