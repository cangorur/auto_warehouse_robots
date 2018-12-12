#include "Math.h"

#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Point.h"
#include "agent/path_planning/GridNode.h"
#include "agent/path_planning/Map.h"

ThetaStarMap::ThetaStarMap(Point mapSize, Map* map, float resolution) :
	map(map) {
	
	Point start(-mapSize.x/2.f + resolution, -mapSize.y/2.f + resolution);
	Point end(mapSize.x/2.f - resolution, mapSize.y/2.f - resolution);

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
}

/*void ThetaStarMap::draw(sf::RenderWindow& renderWindow) {
	sf::CircleShape circleShape(0.2f);
	circleShape.setFillColor(sf::Color(200, 200, 200));
	circleShape.setOrigin(0.2f, 0.2f);

	for(const auto& element : nodes) {
		if(DRAW_EDGES) {
			sf::Vertex line[8 * 2];
			sf::Color edgesColor = sf::Color(0, 255, 0, 40);
			
			int i = 0;
			for(auto neighbour : element.second.neighbours) {
				line[i] = sf::Vertex(element.second.pos, edgesColor);
				line[i + 1] = sf::Vertex(neighbour->pos, edgesColor);
				i += 2;
			}

			renderWindow.draw(line, static_cast<size_t>(i), sf::Lines);
		}
		
		circleShape.setPosition(element.second.pos);
		renderWindow.draw(circleShape);	
	}
}*/

void ThetaStarMap::linkToNode(GridNode& node, Point targetPos) {
	auto iter = nodes.find(targetPos);
	if(iter != nodes.end() && map->isLineOfSightFree(node.pos, targetPos)) {
		node.neighbours.push_back(&iter->second);
	}
}

const GridNode* ThetaStarMap::getNodeClosestTo(Point pos) const {
	float shortestDistance = std::numeric_limits<float>::max();
	const GridNode* nearestNode = nullptr;
	
	for(const auto& element : nodes) {
		float distance = Math::getDistance(element.first, pos);

		if(distance < shortestDistance) {
			shortestDistance = distance;
			nearestNode = &element.second;
		}
	}

	return nearestNode;
}

bool ThetaStarMap::isLineOfSightFree(const Point& pos1, const Point& pos2) const {
	return map->isLineOfSightFree(pos1, pos2);
}
