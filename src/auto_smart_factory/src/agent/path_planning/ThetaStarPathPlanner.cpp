#include <queue>
#include "agent/path_planning/TimedLineOfSightResult.h"

#include "ros/ros.h"
#include "Math.h"
#include "agent/path_planning/ThetaStarPathPlanner.h"

ThetaStarPathPlanner::ThetaStarPathPlanner(ThetaStarMap* thetaStarMap, RobotHardwareProfile* hardwareProfile) :
	map(thetaStarMap),
	hardwareProfile(hardwareProfile) 
{	
}

// TODO  ensure that new infinite reservations dont intersect with existing ones (permanent and time-limited)
// Todo reserve approach routine space

Path ThetaStarPathPlanner::findPath(OrientedPoint start, OrientedPoint target, double startingTime, double targetReservationTime) {
	// Convert to degree
	startPoint = OrientedPoint(start.x, start.y, Math::toDeg(start.o));
	endPoint = OrientedPoint(target.x, target.y, Math::toDeg(target.o));

	map->addAdditionalNode(Point(start.x, target.y));
	map->addAdditionalNode(Point(target.x, target.y));
	
	const GridNode* startNode = map->getNodeClosestTo(Point(start));
	const GridNode* targetNode = map->getNodeClosestTo(Point(target));
	
	if(startNode == nullptr || targetNode == nullptr) {
		if(startNode == nullptr) {
			ROS_FATAL("[Agent %d] StartPoint %f/%f is not in theta* map!", map->getOwnerId(), start.x, start.y);
		}
		if(targetNode == nullptr) {
			ROS_FATAL("[Agent %d] TargetPoint %f/%f is not in theta* map!", map->getOwnerId(), target.x, target.y);
		}
		return Path();
	}
	
	double initialWaitTime = 0;
	TimedLineOfSightResult initialCheckResult = map->whenIsTimedLineOfSightFree(Point(start.x, start.y), startingTime, startNode->pos, startingTime + 0.1f);
	if(initialCheckResult.blockedByTimed) {
		initialWaitTime = initialCheckResult.freeAfter - (startingTime + 0.1f);
		
		if(initialWaitTime > 1000) {
			ROS_FATAL("[Agent %d] Initial wait time > 1000 -> standing in infinite reservation, no valid path possible", map->getOwnerId());
			ROS_WARN("Reservations for start:");
			map->listAllReservationsIn(Point(start.x, start.y));	
			
			return Path();
		} else {
			ROS_WARN("[Agent %d] Path needed initial wait time of %f", map->getOwnerId(), initialWaitTime);
			map->listAllReservationsIn(Point(start.x, start.y));
		}
	}
	initialWaitTime = 0;

	GridInformationMap exploredSet;
	GridInformationPairQueue queue;

	// Push start node
	exploredSet.insert(std::make_pair(startNode->pos, ThetaStarGridNodeInformation(startNode, nullptr, startingTime + initialWaitTime)));
	queue.push(std::make_pair(startingTime + initialWaitTime, &exploredSet.at(startNode->pos)));

	bool targetFound = false;
	ThetaStarGridNodeInformation* targetInformation = nullptr;

	while(!queue.empty()) {
		ThetaStarGridNodeInformation* current = queue.top().second;
		ThetaStarGridNodeInformation* prev = current->prev;
		queue.pop();

		// Target found
		if(current->node == targetNode) {
			targetFound = true;
			targetInformation = current;
			break;
		}

		// Explore all neighbours		
		for(auto neighbourNode : current->node->neighbours) {
			ThetaStarGridNodeInformation* neighbour = &exploredSet.insert(std::make_pair(neighbourNode->pos, ThetaStarGridNodeInformation(neighbourNode, nullptr, initialTime))).first->second;

			// Driving time only includes the additional time to drive from newPrev to neighbour.
			// Therefore: newPrev->time + drivingTime + waitingTime = neighbour->time must be true!
			// Waiting time is the time which must be waited at newPrev before driving can start.

			// Turning time is considered as part of the following line segment. => drivingTime includes previous turning time
			// Therefore, a line segment driving time = Time to turn to target Point + driving time to target point

			double drivingTime = 0;
			double waitingTime = 0;
			ThetaStarGridNodeInformation* newPrev = nullptr;
			bool makeConnection = false;

			// Only try direct connection with prev if not at start node
			bool connectionWithPrevPossible = prev != nullptr;
			if(connectionWithPrevPossible) {
				double timeAtPrev = prev->time;
				double timeAtNeighbour = prev->time + getDrivingTime(prev, neighbour);

				TimedLineOfSightResult result = map->whenIsTimedLineOfSightFree(prev->node->pos, timeAtPrev, neighbour->node->pos, timeAtNeighbour);
				connectionWithPrevPossible = !result.blockedByStatic && !result.blockedByTimed && (!result.hasUpcomingObstacle || (result.hasUpcomingObstacle && timeAtNeighbour < result.lastValidEntryTime));
			}

			if(connectionWithPrevPossible) {
				drivingTime = getDrivingTime(prev, neighbour);
				newPrev = prev;
				makeConnection = true;
			} else {
				double timeAtCurrent = current->time;
				double timeAtNeighbour = current->time + getDrivingTime(current, neighbour);
				TimedLineOfSightResult result = map->whenIsTimedLineOfSightFree(current->node->pos, timeAtCurrent, neighbour->node->pos, timeAtNeighbour);

				if(!result.blockedByStatic) {
					bool waitBecauseUpcomingObstacle = result.hasUpcomingObstacle && timeAtNeighbour >= result.lastValidEntryTime;

					if(!result.blockedByTimed && !waitBecauseUpcomingObstacle) {
						drivingTime = getDrivingTime(current, neighbour);
						newPrev = current;
						makeConnection = true;
					} else {
						// Wait
						if(waitBecauseUpcomingObstacle) {
							waitingTime = result.freeAfterUpcomingObstacle - current->time;
							ROS_ASSERT_MSG(waitingTime >= 0, "waitBecauseUpcomingObstacle waitingTime: %f", waitingTime);
							//printf("Waiting because of upcoming obstacle: %f - Pos: %.1f/%.1f\n", waitingTime, current->node->pos.x, current->node->pos.y);
						} else {
							waitingTime = result.freeAfter - current->time;
							ROS_ASSERT_MSG(waitingTime >= 0, "waitingTime: %f", waitingTime);
							//printf("Waiting because of current obstacle: %f - Pos: %.1f/%.1f - CurrentTime: %f\n", waitingTime, current->node->pos.x, current->node->pos.y, current->time);
						}

						drivingTime = getDrivingTime(current, neighbour);
						newPrev = current;
						makeConnection = true;
					}
				}
			}

			if(makeConnection && (newPrev->time + drivingTime + waitingTime) < neighbour->time) {
				// Check for if connection is valid for upcoming obstacles

				if(map->isTimedConnectionFree(newPrev->node->pos, neighbour->node->pos, newPrev->time, waitingTime, drivingTime) || current->prev == nullptr) {
					double heuristic = getHeuristic(neighbour, targetNode->pos);

					neighbour->time = newPrev->time + drivingTime + waitingTime;
					neighbour->prev = newPrev;
					neighbour->waitTimeAtPrev = waitingTime;
					queue.push(std::make_pair(neighbour->time + heuristic, neighbour));
					
					if(current->prev == nullptr && !map->isTimedConnectionFree(newPrev->node->pos, neighbour->node->pos, newPrev->time, waitingTime, drivingTime)) {
						ROS_WARN("[Agent %d] Only taking connection because at start node", map->getOwnerId());
					}
				}
			}
		}
	}

	if(targetFound) {
		return constructPath(startingTime, targetInformation, initialWaitTime, targetReservationTime);
	} else {
		ROS_WARN("[Agent %d] No path found from node %f/%f to node %f/%f!", map->getOwnerId(), startNode->pos.x,startNode->pos.y, targetNode->pos.x, targetNode->pos.y);
		ROS_WARN("Reservations for start:");
		map->listAllReservationsIn(startNode->pos);

		ROS_WARN("Reservations for target:");
		map->listAllReservationsIn(targetNode->pos);

		return Path();
	}
}

double ThetaStarPathPlanner::getHeuristic(ThetaStarGridNodeInformation* current, Point targetPos) const {
	return hardwareProfile->getDrivingDuration(Math::getDistance(current->node->pos, targetPos));
}

double ThetaStarPathPlanner::getDrivingTime(ThetaStarGridNodeInformation* current, ThetaStarGridNodeInformation* target) const {
	// Include turningTime to current line segment if prev is available
	double turningTime = 0;

	double prevLineSegmentRotation = 0;
	if(current->prev != nullptr) {
		prevLineSegmentRotation = Math::getRotationInDeg(current->node->pos - current->prev->node->pos);		
	} else {
		prevLineSegmentRotation = startPoint.o;
	}

	double currLineSegmentRotation = Math::getRotationInDeg(target->node->pos - current->node->pos);
	turningTime = hardwareProfile->getTurningDuration(std::abs(Math::getAngleDifferenceInDegree(prevLineSegmentRotation, currLineSegmentRotation)));

	return hardwareProfile->getDrivingDuration(Math::getDistance(current->node->pos, target->node->pos)) + turningTime;
}

// Todo add estimated turning times to path reservation generation

Path ThetaStarPathPlanner::constructPath(double startingTime, ThetaStarGridNodeInformation* targetInformation, double initialWaitTime, double targetReservationTime) const {
	std::vector<Point> pathNodes;
	std::vector<double> waitTimes;
	ThetaStarGridNodeInformation* currentGridInformation = targetInformation;
	double waitTimeAtPrev = 0;
	
	int i = 0;
	while(currentGridInformation != nullptr) {
		pathNodes.emplace_back(currentGridInformation->node->pos);
		ROS_ASSERT_MSG(waitTimeAtPrev >= 0, "waitTimeAtPrev: %f", waitTimeAtPrev);
		waitTimes.push_back(waitTimeAtPrev);

		waitTimeAtPrev = currentGridInformation->waitTimeAtPrev;
		currentGridInformation = currentGridInformation->prev;
		
		if(i++ > 150) {
			ROS_FATAL("[Agent %d] Endless loop in construct path => aborting. Start: %f/%f Target: %f/%f", map->getOwnerId(), startPoint.x, startPoint.y, endPoint.x, endPoint.y);
			return Path();
		}
	}

	std::reverse(pathNodes.begin(), pathNodes.end());
	std::reverse(waitTimes.begin(), waitTimes.end());

	// Convert start/end orientation to rad
	OrientedPoint startOrientation = startPoint;
	startOrientation.o = Math::toRad(startOrientation.o);

	OrientedPoint endOrientation = endPoint;
	endOrientation.o = Math::toRad(endOrientation.o);
	
	return Path(startingTime, pathNodes, waitTimes, hardwareProfile, targetReservationTime, startOrientation, endOrientation);
}

