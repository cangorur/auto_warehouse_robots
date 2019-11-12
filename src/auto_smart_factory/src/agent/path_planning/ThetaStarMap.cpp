
#include <include/agent/path_planning/ThetaStarMap.h>

#include "agent/path_planning/ThetaStarMap.h"

#include "ros/ros.h"
#include "agent/path_planning/Map.h"

ThetaStarMap::ThetaStarMap(Map* map, float resolution) :
	map(map),
	resolution(resolution)
{
	Point start(map->getMargin(), map->getMargin());
	Point end(map->getWidth() - map->getMargin(), map->getHeight() - map->getMargin());

	// Generate
	Point current = start;
	while(current.x <= end.x) {
		current.y = start.y;
		while(current.y <= end.y) {
			if(!map->isInsideAnyStaticInflatedObstacle(current)) {
				nodes.emplace(std::pair<Point, GridNode*>(current, new GridNode(current)));
			}
			current.y += resolution;
		}
		current.x += resolution;
	}

	// Link
	for(const auto& element : nodes) {
		linkToNode(element.second, element.second->pos + Point(- resolution, 0));
		linkToNode(element.second, element.second->pos + Point(- resolution, - resolution));
		linkToNode(element.second, element.second->pos + Point(- resolution, + resolution));
		linkToNode(element.second, element.second->pos + Point(0, - resolution));
		linkToNode(element.second, element.second->pos + Point(0, + resolution));
		linkToNode(element.second, element.second->pos + Point(+ resolution, 0));
		linkToNode(element.second, element.second->pos + Point(+ resolution, - resolution));
		linkToNode(element.second, element.second->pos + Point(+ resolution, + resolution));
	}

	// Add point in front of tray
	for(const auto& tray : map->getTrays()) {
		OrientedPoint op = map->getPointInFrontOfTray(tray);
		Point p(op);
		auto nodes_pair = nodes.emplace(std::pair<Point, GridNode*>(p, new GridNode(p)));
		GridNode* node = (nodes_pair.first)->second;
		
		Point target;
		float offset = 1.0f;
		double inputDx = std::cos(op.o);
		double inputDy = std::sin(op.o);
		target.x = static_cast<float>(op.x - offset * inputDx);
		target.y = static_cast<float>(op.y - offset * inputDy);

		GridNode* closestNode = getNodeClosestTo(target);

		node->neighbours.push_back(closestNode);
		closestNode->neighbours.push_back(node);	
	}
}

void ThetaStarMap::linkToNode(GridNode* node, Point targetPos) {
	auto iter = nodes.find(targetPos);
	if(iter != nodes.end() && map->isStaticLineOfSightFree(node->pos, targetPos)) {
		node->neighbours.push_back(iter->second);
	}
}

const GridNode* ThetaStarMap::getNodeClosestTo(const Point& pos) const {
	double shortestDistance = std::numeric_limits<float>::max();
	const GridNode* nearestNode = nullptr;
	
	for(const auto& element : nodes) {
		double distance = Math::getDistanceSquared(element.first, pos);

		if(distance < shortestDistance) {
			shortestDistance = distance;
			nearestNode = element.second;
		}
	}

	if(nearestNode == nullptr) {
		ROS_ERROR("No nearest node found!");
	}
	
	return nearestNode;
}

GridNode* ThetaStarMap::getNodeClosestTo(const Point& pos) {
	double shortestDistance = std::numeric_limits<float>::max();
	GridNode* nearestNode = nullptr;
	
	for(const auto& element : nodes) {
		double distance = Math::getDistanceSquared(element.first, pos);

		if(distance < shortestDistance) {
			shortestDistance = distance;
			nearestNode = element.second;
		}
	}

	if(nearestNode == nullptr) {
		ROS_ERROR("No nearest node found!");
	}
	
	return nearestNode;
}

TimedLineOfSightResult ThetaStarMap::whenIsTimedLineOfSightFree(const Point& pos1, double startTime, const Point& pos2, double endTime, const std::vector<Rectangle>& smallerReservations) const {
	return map->whenIsTimedLineOfSightFree(pos1, startTime, pos2, endTime, smallerReservations);
}

bool ThetaStarMap::isTimedConnectionFree(const Point& pos1, const Point& pos2, double startTime, double waitingTime, double drivingTime, const std::vector<Rectangle>& smallerReservations) const {
	return map->isTimedConnectionFree(pos1, pos2, startTime, waitingTime, drivingTime, smallerReservations);
}

bool ThetaStarMap::addAdditionalNode(Point pos) {
	if(!map->isPointInMap(pos) || map->isInsideAnyStaticInflatedObstacle(pos)) {
		return false;
	}

	// Add new node
	GridNode* newGridNode = new GridNode(pos);
	auto emplaceResult = nodes.emplace(std::pair<Point, GridNode*>(pos, newGridNode));

	// If node was newly added
	if(emplaceResult.second) {
		double maxDistance = resolution * resolution;

		for(std::pair<Point, GridNode*> neighbour : nodes) {
			double distance = Math::getDistanceSquared(neighbour.first, pos);

			if(distance <= maxDistance && map->isStaticLineOfSightFree(pos, neighbour.first))  {
				newGridNode->neighbours.push_back(neighbour.second);
				neighbour.second->neighbours.push_back(newGridNode);
			}
		}
	} else {
		delete newGridNode;
	}
	
	return true;
}

int ThetaStarMap::getOwnerId() const {
	return map->getOwnerId();
}

std::vector<Rectangle> ThetaStarMap::getRectanglesOnStartingPoint(Point p) const {
	return map->getRectanglesOnStartingPoint(p);
}

visualization_msgs::Marker ThetaStarMap::getGridVisualization() {
	visualization_msgs::Marker msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.ns = "Grid";
	msg.action = visualization_msgs::Marker::MODIFY;
	msg.pose.orientation.w = 1.0;

	msg.id = 0;
	msg.type = visualization_msgs::Marker::SPHERE_LIST;

	msg.scale.x = 0.1f;
	msg.scale.y = 0.1f;
	msg.scale.z = 0.1f;

	msg.color.r = 0.1f;
	msg.color.g = 0.1f;
	msg.color.b = 0.1f;
	msg.color.a = 0.3;

	geometry_msgs::Point p;
	p.z = 0.f;

	// Nodes
	int i = 0;
	for(const auto& node : nodes) {
		const Point* point = &node.first;

		p.x = point->x;
		p.y = point->y;
		msg.points.push_back(p);
	}


	return msg;	
}

visualization_msgs::Marker ThetaStarMap::getLinkVisualization() {
	visualization_msgs::Marker msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.ns = "Links";
	msg.action = visualization_msgs::Marker::MODIFY;
	msg.pose.orientation.w = 1.0;

	msg.id = 0;
	msg.type = visualization_msgs::Marker::LINE_LIST;

	msg.scale.x = 0.02f;
	msg.scale.y = 0.02f;
	msg.scale.z = 0.02f;

	msg.color.r = 0.1f;
	msg.color.g = 0.1f;
	msg.color.b = 0.1f;
	msg.color.a = 0.15;

	geometry_msgs::Point p;
	p.z = 0.f;

	// Nodes
	int i = 0;
	for(const auto& node : nodes) {
		const Point* start = &node.first;

		for(const auto& neighbour : node.second->neighbours) {
			const Point* end = &neighbour->pos;

			p.x = start->x;
			p.y = start->y;
			msg.points.push_back(p);

			p.x = end->x;
			p.y = end->y;
			msg.points.push_back(p);
		}
	}


	return msg;	
}