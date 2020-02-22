#include <queue>
#include "agent/path_planning/TimedLineOfSightResult.h"
#include "ros/ros.h"
#include "Math.h"
#include "agent/path_planning/ThetaStarPathPlanner.h"

using namespace UncertaintyDirection;

ThetaStarPathPlanner::ThetaStarPathPlanner(ThetaStarMap* thetaStarMap, RobotHardwareProfile* hardwareProfile, OrientedPoint start, OrientedPoint target, double startingTime, double targetReservationTime, bool ignoreStartingReservations) :
	map(thetaStarMap),
	hardwareProfile(hardwareProfile),
	start(OrientedPoint(start.x, start.y, Math::toDeg(start.o))),
	target(OrientedPoint(target.x, target.y, Math::toDeg(target.o))),
	startingTime(startingTime),
	targetReservationTime(targetReservationTime),
	timing(startingTime, start, hardwareProfile)
{
	isValidPathQuery = true;
	
	/*
	if(!map->addAdditionalNode(Point(start.x, start.y))) {
		ROS_FATAL("[Agent %d] Node at StartPoint %f/%f could not be created. Taking nearest other point instead", map->getOwnerId(), start.x, start.y);
	}
	if(!map->addAdditionalNode(Point(target.x, target.y))) {
		ROS_FATAL("[Agent %d] Node at TargetPoint %f/%f could not be created. Taking nearest other point instead", map->getOwnerId(), target.x, target.y);
	}
	*/

	startNode = map->getNodeClosestTo(Point(start));
	targetNode = map->getNodeClosestTo(Point(target));
	
	if(startNode == nullptr) {
		ROS_FATAL("[Agent %d] StartPoint %f/%f is not in theta* map!", map->getOwnerId(), start.x, start.y);
		isValidPathQuery = false;
	}
	if(targetNode == nullptr) {
		ROS_FATAL("[Agent %d] TargetPoint %f/%f is not in theta* map!", map->getOwnerId(), target.x, target.y);
		isValidPathQuery = false;
	}	

	double initialWaitTime = 0;
	// Use empty vector here
	smallerReservations.clear();
	TimedLineOfSightResult initialCheckResult = map->whenIsTimedLineOfSightFree(startNode->pos, startingTime, startNode->pos, startingTime + 1.1f, smallerReservations);
	if(initialCheckResult.blockedByTimed) {
		initialWaitTime = initialCheckResult.freeAfter - (startingTime + 0.1f);

		if(ignoreStartingReservations) {
			smallerReservations = map->getRectanglesOnStartingPoint(startNode->pos);
			ROS_WARN("[Agent %d] Initial wait time of %f. Using %d smaller reservations instead!", map->getOwnerId(), initialWaitTime, (int) smallerReservations.size());	
		} else {
			//ROS_WARN("[Agent %d] Path would need initial wait time of %f", map->getOwnerId(), initialWaitTime);
			isValidPathQuery = false;
		}			
	}
}

Path ThetaStarPathPlanner::findPath() {
	if(!isValidPathQuery) {
		return Path();
	}
	
	if(start.x == target.x && start.y == target.y) {
		return Path(startingTime, {Point(start), Point(start)}, {0.0, 0.0}, hardwareProfile, targetReservationTime, start, start, map->getOwnerId());
	}
	
	GridInformationMap exploredSet;
	GridInformationPairQueue queue;

	// Push start node
	exploredSet.insert(std::make_pair(startNode->pos, ThetaStarGridNodeInformation(startNode, nullptr, startingTime)));
	queue.push(std::make_pair(startingTime, &exploredSet.at(startNode->pos)));

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

			// Only try direct connection with prev if not at start node (prev != nullptr) AND not blocked by timed obstacle
			bool prevNotNull = prev != nullptr;
			bool connectionWithPrevPossible = false;
			if(prevNotNull) {
				double timeAtPrev = prev->time;
				timeAtPrev -= timing.getPlanningUncertainty(timeAtPrev, Direction::BEHIND);
				double timeAtNeighbour = prev->time + timing.getDrivingAndTurningTime(prev, neighbour);
				timeAtNeighbour += timing.getPlanningUncertainty(timeAtNeighbour, Direction::AHEAD);

				TimedLineOfSightResult result = map->whenIsTimedLineOfSightFree(prev->node->pos, timeAtPrev, neighbour->node->pos, timeAtNeighbour, smallerReservations);
				
				connectionWithPrevPossible = !result.blockedByStatic && !result.blockedByTimed && (!result.hasUpcomingObstacle || (result.hasUpcomingObstacle && timeAtNeighbour < result.lastValidEntryTime));
			}

			if(prevNotNull && connectionWithPrevPossible) {
				drivingTime = timing.getDrivingAndTurningTime(prev, neighbour);
				newPrev = prev;
				makeConnection = true;
			} else {
				// If no direct connection possible, try to connect via current
				double timeAtCurrent = current->time;
				timeAtCurrent -= timing.getPlanningUncertainty(timeAtCurrent, Direction::BEHIND);
				double timeAtNeighbour = current->time + timing.getDrivingAndTurningTime(current, neighbour);
				timeAtNeighbour += timing.getPlanningUncertainty(timeAtNeighbour, Direction::AHEAD);
				TimedLineOfSightResult result = map->whenIsTimedLineOfSightFree(current->node->pos, timeAtCurrent, neighbour->node->pos, timeAtNeighbour, smallerReservations);

				if(!result.blockedByStatic) {
					bool waitBecauseUpcomingObstacle = result.hasUpcomingObstacle && timeAtNeighbour >= result.lastValidEntryTime;

					if(!result.blockedByTimed && !waitBecauseUpcomingObstacle) {
						drivingTime = timing.getDrivingAndTurningTime(current, neighbour);
						newPrev = current;
						makeConnection = true;
					} else {
						// Calculate wait time
						if(waitBecauseUpcomingObstacle) {
							waitingTime = result.freeAfterUpcomingObstacle - current->time;
						} else {
							waitingTime = result.freeAfter - current->time;
						}
						waitingTime = std::max(0.0, waitingTime);
						//waitingTime += timing.getRelativeUncertainty(waitingTime);

						drivingTime = timing.getDrivingAndTurningTime(current, neighbour);
						newPrev = current;
						makeConnection = true;
					}
				}
			}

			// Finally try to make connection
			if(makeConnection && (newPrev->time + drivingTime + waitingTime) < neighbour->time) {
				// Check for if connection is valid for upcoming obstacles
				if(map->isTimedConnectionFree(newPrev->node->pos, neighbour->node->pos, newPrev->time, waitingTime, drivingTime, smallerReservations)) {
					double heuristic = getHeuristic(neighbour, targetNode->pos);

					neighbour->time = newPrev->time + drivingTime + waitingTime;
					neighbour->prev = newPrev;
					neighbour->waitTimeAtPrev = waitingTime;
					queue.push(std::make_pair(neighbour->time + heuristic, neighbour));
				} else {
					TimedLineOfSightResult result = map->whenIsTimedLineOfSightFree(newPrev->node->pos, newPrev->time, neighbour->node->pos, newPrev->time + waitingTime + drivingTime, smallerReservations);
					
					if(!result.blockedByStatic && result.blockedByTimed) {
						double newWaitingTime = result.freeAfter - newPrev->time;
						waitingTime = std::max(waitingTime, newWaitingTime);

						if(map->isTimedConnectionFree(newPrev->node->pos, neighbour->node->pos, newPrev->time, waitingTime, drivingTime, smallerReservations)) {
							double heuristic = getHeuristic(neighbour, targetNode->pos);

							neighbour->time = newPrev->time + drivingTime + waitingTime;
							neighbour->prev = newPrev;
							neighbour->waitTimeAtPrev = waitingTime;
							queue.push(std::make_pair(neighbour->time + heuristic, neighbour));
							//ROS_INFO("[Agent %d] Made connection only after second attempt", map->getOwnerId());
						}
					}
				}
			}
		}
	}

	if(targetFound) {
		Path path = constructPath(startingTime, targetInformation, targetReservationTime);
		return smoothPath(path);
	} else {
		//ROS_WARN("[Agent %d] No path found from node %f/%f to node %f/%f!", map->getOwnerId(), startNode->pos.x,startNode->pos.y, targetNode->pos.x, targetNode->pos.y);
		//ROS_WARN("Reservations for start:");
		//map->listAllReservationsIn(startNode->pos);

		//ROS_WARN("Reservations for target:");
		//map->listAllReservationsIn(targetNode->pos);

		return Path();
	}
}

double ThetaStarPathPlanner::getHeuristic(ThetaStarGridNodeInformation* current, Point targetPos) const {
	return hardwareProfile->getDrivingDuration(Math::getDistance(current->node->pos, targetPos));
}

Path ThetaStarPathPlanner::constructPath(double startingTime, ThetaStarGridNodeInformation* targetInformation, double targetReservationTime) const {
	std::vector<Point> pathNodes;
	std::vector<double> waitTimes;
	ThetaStarGridNodeInformation* currentGridInformation = targetInformation;
	double waitTimeAtPrev = 0;
	
	int i = 0;
	while(currentGridInformation != nullptr) {
		pathNodes.emplace_back(currentGridInformation->node->pos);
		waitTimes.push_back(waitTimeAtPrev);

		waitTimeAtPrev = currentGridInformation->waitTimeAtPrev;
		currentGridInformation = currentGridInformation->prev;
		
		if(i++ > 100) {
			ROS_FATAL("[Agent %d] Endless loop in construct path => aborting. Start: %f/%f Target: %f/%f", map->getOwnerId(), start.x, start.y, target.x, target.y);
			return Path();
		}
	}

	std::reverse(pathNodes.begin(), pathNodes.end());
	std::reverse(waitTimes.begin(), waitTimes.end());
	
	if(pathNodes.size() <= 1) {
		ROS_FATAL("[Agent %d] PathNodesSize: %d", map->getOwnerId(), (int) pathNodes.size());
		ROS_WARN("Start: %f/%f | Target: %f/%f", start.x, start.y, target.x, target.y);
		for(int j = 0; j < pathNodes.size(); j++) {
			ROS_WARN("[%d] %f/%f", j, pathNodes[j].x, pathNodes[j].y);	
		}	
		ROS_ASSERT(pathNodes.size() > 1);
	}

	// Convert orientation to rad
	return Path(startingTime, pathNodes, waitTimes, hardwareProfile, targetReservationTime, OrientedPoint(start.x, start.y, Math::toRad(start.o)), OrientedPoint(target.x, target.y, Math::toRad(target.o)), map->getOwnerId());
}

Path ThetaStarPathPlanner::smoothPath(Path inputPath) const {
	if(inputPath.getNodes().size() < 3) {
		return inputPath;
	}
	const std::vector<Point>& input = inputPath.getNodes();
	const std::vector<double>& inputWaitTimes = inputPath.getWaitTimes();
	std::vector<Point> output;
	std::vector<double> outputWaitTimes;

	output.push_back(input[0]);
	outputWaitTimes.push_back(inputWaitTimes[0]);

	for(unsigned long i = 1; i < input.size() - 1; ) {
		Point prev = input[i - 1];
		Point curr = input[i];
		Point next = input[i + 1];

		if(shouldSmoothCorner(prev, curr, next, inputWaitTimes[i])) {
			std::vector<Point> subcurve = createCurveFromCorner(prev, curr, next, output.back());

			// Only add points if not prev/next to avoid adding them twice (in cases where |prev,curr| is so small that prev is part of smoothed curve
			for(auto& v : subcurve) {
				if(v != prev && v != next && v != output.back()) {
					output.push_back(v);
					outputWaitTimes.push_back(0);
				}
			}
		} else {
			// Push back current
			output.push_back(curr);
			outputWaitTimes.push_back(inputWaitTimes[i]);
		}

		i++;
	}

	output.push_back(input.back());
	outputWaitTimes.push_back(inputWaitTimes.back());

	return Path(inputPath.getStartTimeOffset(), output, outputWaitTimes, hardwareProfile, targetReservationTime, inputPath.getStart(), inputPath.getEnd(), map->getOwnerId());
}

std::vector<Point> ThetaStarPathPlanner::createCurveFromCorner(Point prev, Point curr, Point next, Point lastPointInOutput) const {
	// Return curve which replaces the center input point
	int pointsToInsert = 1;

	Point curveStart = getCurveEdge(prev, curr, lastPointInOutput);
	Point curveEnd = getCurveEdge(next, curr);

	// Creates a curve with start, center and end points, possibly more
	std::vector<Point> curve = addPointsToCurve(curveStart, curr, curveEnd, pointsToInsert);

	// Smooth
	curve = smoothCurve(curve);

	return curve;
}

std::vector<Point> ThetaStarPathPlanner::smoothCurve(const std::vector<Point>& input) const {
	std::vector<Point> output = input;

	double weightChange = 0.55f;
	double weightSmooth = 0.4f;
	double tolerance = 0.5f;

	double change = tolerance;
	int iterations = 0;
	while(change >= tolerance && iterations < 20) {
		change = 0.f;
		iterations++;
		for(unsigned long i = 1; i < input.size() - 1; i++) {
			// X
			double inX = input.at(i).x;
			double outX = output.at(i).x;
			double outPrevX = output.at(i - 1).x;
			double outNextX = output.at(i + 1).x;

			double outSavedX = outX;
			outX += weightChange * (inX - outX) + weightSmooth * (outNextX + outPrevX - (2.f * outX));
			output.at(i).x = outX;

			change += std::abs(outX - outSavedX);

			// Y
			double inY = input.at(i).y;
			double outY = output.at(i).y;
			double outPrevY = output.at(i - 1).y;
			double outNextY = output.at(i + 1).y;

			double outSavedY = outY;
			outY += weightChange * (inY - outY) + weightSmooth * (outNextY + outPrevY - (2.f * outY));
			output.at(i).y = outY;

			change += std::abs(outY - outSavedY);
		}
	}

	return output;
}

double ThetaStarPathPlanner::getAngle(const Point& prev, const Point& curr, const Point& next) const {
	double rotationA = Math::getRotationInDeg(curr - prev);
	double rotationB = Math::getRotationInDeg(next - curr);

	return std::abs(Math::getAngleDifferenceInDegree(rotationA, rotationB));
}

Point ThetaStarPathPlanner::getCurveEdge(Point neighbour, Point center) const {
	double distance = Math::getDistance(neighbour, center);
	if(distance > desiredDistanceForCurveEdge) {
		return center + (neighbour - center)/distance * desiredDistanceForCurveEdge;
	} else {
		return neighbour;
	}
}

Point ThetaStarPathPlanner::getCurveEdge(Point neighbour, Point center, Point lastPointInOutput) const {
	double distance = Math::getDistance(neighbour, center);
	Point curvePoint;
	if(distance > desiredDistanceForCurveEdge) {
		curvePoint = center + (neighbour - center)/distance * desiredDistanceForCurveEdge;
	} else {
		curvePoint = neighbour;
	}

	if(Math::getDistance(center, curvePoint) < Math::getDistance(center, lastPointInOutput)) {
		return curvePoint;
	} else {
		return lastPointInOutput;
	}
}

std::vector<Point> ThetaStarPathPlanner::addPointsToCurve(Point curveStart, Point center, Point curveEnd, int pointsToAdd) const {
	std::vector<Point> output;
	Point offset;

	if(pointsToAdd == 0) {		output.push_back(curveStart);
		output.push_back(center);
		output.push_back(curveEnd);
		return output;
	}

	output.push_back(curveStart);

	// First half	
	offset = (center - curveStart) / (double)(pointsToAdd + 1);
	for(int i = 0; i < pointsToAdd; i++) {
		output.push_back(curveStart + (double)(i + 1) * offset);
	}

	output.push_back(center);

	// Second half
	offset = (curveEnd - center) / (double)(pointsToAdd + 1);
	for(int i = 0; i < pointsToAdd; i++) {
		output.push_back(center + (double)(i + 1) * offset);
	}

	output.push_back(curveEnd);

	return output;
}

bool ThetaStarPathPlanner::shouldSmoothCorner(Point prev, Point curr, Point next, double waitTimeAtCenter) const {
	double angle = getAngle(prev, curr, next);
	return angle >= 35.f && waitTimeAtCenter == 0;
}


