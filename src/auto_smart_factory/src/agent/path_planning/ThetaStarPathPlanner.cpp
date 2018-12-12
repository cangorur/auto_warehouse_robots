#include <iostream>
#include <queue>

#include "Math.h"
#include "agent/path_planning/ThetaStarPathPlanner.h"

ThetaStarPathPlanner::ThetaStarPathPlanner(ThetaStarMap* thetaStarMap) :
	map(thetaStarMap) {	

}

Path ThetaStarPathPlanner::findPath(Point start, Point target) {
	const GridNode* startNode = map->getNodeClosestTo(start);
	const GridNode* targetNode = map->getNodeClosestTo(target);

	if(startNode == nullptr || targetNode == nullptr) {
		std::cout << "Error: Start or Target not in ThetaStar map!" << std::endl;
		return Path({});
	}

	GridInformationMap exploredSet;
	GridInformationPairQueue queue;

	// Push start node
	exploredSet.insert(std::make_pair(startNode->pos, ThetaStarGridNodeInformation(startNode, nullptr, 0)));
	queue.push(std::make_pair(getHeuristic(startNode->pos, targetNode->pos) + 0, &exploredSet.at(startNode->pos)));
	
	bool targetFound = false;
	ThetaStarGridNodeInformation* targetInformation = nullptr;

	while(!queue.empty()) {
		ThetaStarGridNodeInformation* current = queue.top().second;
		ThetaStarGridNodeInformation* prev = current->prev;
		queue.pop();

		// Delayed Line-of-Sight check
		if(prev != nullptr && USE_THETA_STAR && !map->isLineOfSightFree(current->node->pos, prev->node->pos)) {
			// Line of sight not free
			current->prev = current->expandedBy;
			current->g = current->expandedBy->g + Math::getDistance(current->expandedBy->node->pos, current->node->pos);

			float heuristic = getHeuristic(current->node->pos, targetNode->pos);
			queue.push(std::make_pair(current->g + heuristic, current));
			continue;
		}

		// Target found
		if(current->node == targetNode) {
			targetFound = true;
			targetInformation = current;
			break;
		}
		
		// Explore all neighbours		
		for(auto neighbourNode : current->node->neighbours) {
			ThetaStarGridNodeInformation* neighbour = &exploredSet.insert(std::make_pair(neighbourNode->pos, ThetaStarGridNodeInformation(neighbourNode, nullptr, std::numeric_limits<float>::max()))).first->second;

			float newDistance;
			ThetaStarGridNodeInformation* newPrev;
			ThetaStarGridNodeInformation* newExpandedBy;
			
			if(prev != nullptr && USE_THETA_STAR) {
				newDistance = prev->g + Math::getDistance(prev->node->pos, neighbour->node->pos);
				newPrev = prev;
				newExpandedBy = current;
			} else {
				// Start node, only take direct path
				newDistance = current->g + Math::getDistance(current->node->pos, neighbour->node->pos);
				newPrev = current;
				newExpandedBy = nullptr;
			}
			
			if(newDistance < neighbour->g) {
				float heuristic = getHeuristic(neighbour->node->pos, targetNode->pos);
				
				neighbour->g = newDistance;
				neighbour->prev = newPrev;
				neighbour->expandedBy = newExpandedBy;
				queue.push(std::make_pair(newDistance + heuristic, neighbour));
			}
		}
	}
	
	if(targetFound) {
		std::vector<Point> pathNodes;
		ThetaStarGridNodeInformation* currentGridInformation = targetInformation;

		while(currentGridInformation != nullptr) {
			pathNodes.emplace_back(currentGridInformation->node->pos);
			currentGridInformation = currentGridInformation->prev;
		}
		std::reverse(pathNodes.begin(), pathNodes.end());

		return Path(pathNodes);
	} else {
		std::cout << "No path found!" << std::endl;
		return Path({});	
	}	
}

float ThetaStarPathPlanner::getHeuristic(Point point, Point target) const {
	return Math::getDistance(point, target);
}
