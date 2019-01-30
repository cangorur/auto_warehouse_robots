#include "Math.h"

#include "ros/ros.h"
#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Point.h"
#include "agent/path_planning/GridNode.h"
#include "agent/path_planning/Map.h"

ThetaStarMap::ThetaStarMap(Map* map, float resolution) :
	map(map) {
	
	Point start(map->getMargin(), map->getMargin());
	Point end(map->getWidth() - map->getMargin(), map->getHeight() - map->getMargin());

	// Generate
	Point current = start;
	while(current.x <= end.x) {
		current.y = start.y;
		while(current.y <= end.y) {
			if(!map->isInsideAnyInflatedObstacle(current)) {
				nodes.emplace(std::pair<Point, GridNode>(current, GridNode(current)));
			}
			current.y += resolution;
		}
		current.x += resolution;
	}

	// Link
	for(auto& element : nodes) {
		linkToNode(element.second, element.second.pos + Point(- resolution, 0));
		linkToNode(element.second, element.second.pos + Point(- resolution, - resolution));
		linkToNode(element.second, element.second.pos + Point(- resolution, + resolution));
		linkToNode(element.second, element.second.pos + Point(0, - resolution));
		linkToNode(element.second, element.second.pos + Point(0, + resolution));
		linkToNode(element.second, element.second.pos + Point(+ resolution, 0));
		linkToNode(element.second, element.second.pos + Point(+ resolution, - resolution));
		linkToNode(element.second, element.second.pos + Point(+ resolution, + resolution));
	}
	
	ROS_INFO("[Theta*] Generated map with %d nodes", (int) nodes.size());
}

void ThetaStarMap::linkToNode(GridNode& node, Point targetPos) {
	auto iter = nodes.find(targetPos);
	if(iter != nodes.end() && map->isStaticLineOfSightFree(node.pos, targetPos)) {
		node.neighbours.push_back(&iter->second);
	}
}

const GridNode* ThetaStarMap::getNodeClosestTo(const Point& pos) const {
	float shortestDistance = std::numeric_limits<float>::max();
	const GridNode* nearestNode = nullptr;
	
	for(const auto& element : nodes) {
		float distance = Math::getDistanceSquared(element.first, pos);

		if(distance < shortestDistance) {
			shortestDistance = distance;
			nearestNode = &element.second;
		}
	}

	if(nearestNode == nullptr) {
		ROS_ERROR("No nearest node found!");
	}
	
	return nearestNode;
}

bool ThetaStarMap::isStaticLineOfSightFree(const Point& pos1, const Point& pos2) const {
	return map->isStaticLineOfSightFree(pos1, pos2);
}

bool ThetaStarMap::isTimedLineOfSightFree(const Point& pos1, double startTime, const Point& pos2, double endTime) const {
	return map->isTimedLineOfSightFree(pos1, startTime, pos2, endTime);
}

TimedLineOfSightResult ThetaStarMap::whenIsTimedLineOfSightFree(const Point& pos1, double startTime, const Point& pos2, double endTime) const {
	return map->whenIsTimedLineOfSightFree(pos1, startTime, pos2, endTime);
}

bool ThetaStarMap::isTimedConnectionFree(const Point& pos1, const Point& pos2, double startTime, double waitingTime, double drivingTime) const {
	return map->isTimedConnectionFree(pos1, pos2, startTime, waitingTime, drivingTime);
}
