#include <utility>
#include <cmath>
#include "Math.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "agent/path_planning/Path.h"
#include "agent/path_planning/Map.h"

using namespace UncertaintyDirection;

Path::Path(double startTimeOffset, std::vector<Point> nodes_, std::vector<double> waitTimes_, RobotHardwareProfile* hardwareProfile, double targetReservationTime, OrientedPoint start, OrientedPoint end) :
		startTimeOffset(startTimeOffset),
		nodes(std::move(nodes_)),
		waitTimes(std::move(waitTimes_)),
		hardwareProfile(hardwareProfile),
		targetReservationTime(targetReservationTime),
		start(start),
		end(end),
		isValidPath(true),
		timing(startTimeOffset, start, hardwareProfile)
{
	if(nodes.size() != waitTimes.size()) {
		ROS_FATAL("nodes.size() != waitTimes.size()");
		isValidPath = false;
	}
	if(nodes.size() < 2) {
		ROS_FATAL("Tried to construct path with no node points");
		isValidPath = false;
	}
	if(nodes.at(0) == nodes.at(1)) {
		ROS_FATAL("Tried to construct path with identical points (point[0] == point[1]");
		isValidPath = false;
	}
	
	if(!isValidPath) {
		return;
	}

	// Turning time is considered as part of the following line segment.
	// Therefore, a line segment driving time = Time to turn to target Point + driving time to target point 

	/*duration = waitTimes.at(0);
	batteryConsumption = hardwareProfile->getIdleBatteryConsumption(waitTimes.at(0));
	departureTimes.push_back(startTimeOffset + waitTimes.at(0));
	turningTimes.push_back(getInitialTurningTime());*/
	duration = 0;
	distance = 0;
	batteryConsumption = 0;

	for(unsigned long i = 0; i < nodes.size(); i++) { // last waitTime is not used
		double currentDistance = Math::getDistance(nodes.at(i - 1), nodes.at(i));

		// Don't compute angle difference for last node
		double turningDuration = 0;
		bool performsOnSpotTurn = false;
		if(i == 0) {
			turningDuration = timing.getTurningTime(Math::toDeg(start.o), nodes.at(0), nodes.at(1));
			performsOnSpotTurn = timing.performsOnSpotTurn(Math::toDeg(start.o), nodes.at(0), nodes.at(1));
		} else if(i < nodes.size() - 1) {
			turningDuration = timing.getTurningTime(nodes[i - 1], nodes[i], nodes[i + 1]);
			performsOnSpotTurn = timing.performsOnSpotTurn(nodes[i - 1], nodes[i], nodes[i + 1]);
		}

		double onSpotTime;
		double drivingTime;
		if(performsOnSpotTurn) {
			onSpotTime = std::max(waitTimes.at(i), turningDuration); 
			drivingTime = hardwareProfile->getDrivingDuration(distance);
		} else {
			onSpotTime = waitTimes.at(i);
			drivingTime = hardwareProfile->getDrivingDuration(distance) + turningDuration;
		}
		
		// Distance
		distance += currentDistance;
		
		// Duration: Driving is part of the next segment, add after departureTime
		duration += onSpotTime;
		departureTimes.push_back(startTimeOffset + duration);
		duration += drivingTime;
		
		batteryConsumption += hardwareProfile->getIdleBatteryConsumption(onSpotTime) + hardwareProfile->getDrivingBatteryConsumption(currentDistance);
		
		onSpotTimes.push_back(onSpotTime);
		drivingTimes.push_back(drivingTime);
	}
}

Path::Path() : 
	isValidPath(false)
{
}

const std::vector<Rectangle> Path::generateReservations(int ownerId) const {
	std::vector<Rectangle> reservations;

	double reservationSize = ROBOT_RADIUS * 2.0f;
	Point waitingReservationSize = Point(reservationSize, reservationSize) * 0.98f;

	double currentTime = startTimeOffset;
	bool skippedLastSegment = false;
	for(unsigned int i = 0; i < nodes.size() - 1; i++) {
		double currentDistance = Math::getDistance(nodes[i], nodes[i + 1]);
		Point normalizedDir = (nodes[i + 1] - nodes[i]) * 1.f * (1.f/currentDistance);
		double currentRotation = Math::getRotationInDeg(normalizedDir);

		// OnSpotTime
		if(onSpotTimes.at(0) > 0 || i == 0) {
			double startTime = currentTime - timing.getUncertaintyForReservation(currentTime, Direction::BEHIND) - reservationTimeMarginBehind;
			double endTime = currentTime + onSpotTimes[i] + timing.getUncertaintyForReservation(currentTime + onSpotTimes[i], Direction::AHEAD) + reservationTimeMarginAhead;

			reservations.emplace_back(nodes[i], waitingReservationSize, 0, startTime, endTime, ownerId);
			currentTime += onSpotTimes[i];
		}

		int currentNode = i;
		double currentDrivingTime = drivingTimes.at(i);
		// Skipp line segments if they are too small to avoid generating LOTS of reservations
		/*if(!skippedLastSegment && onSpotTimes.at(i) == 0 && onSpotTimes.at(i + 1) == 0 && currentDistance < 0.15f) {
			skippedLastSegment = true;
			continue;
		}
		
		if(skippedLastSegment) {
			skippedLastSegment = false;
			currentNode -= 1;
			currentDistance = Math::getDistance(nodes[i - 1], nodes[i + 1]);
		}*/

		// Line segment
		auto segmentCount = static_cast<unsigned int>(std::ceil(drivingTimes.at(i) / maxDrivingReservationDuration));
		double deltaTime = currentDrivingTime / static_cast<double>(segmentCount);
		double deltaDistance = currentDistance / static_cast<double>(segmentCount);

		for(unsigned int segmentIndex = 0; segmentIndex < segmentCount; segmentIndex++) {
			auto alpha = static_cast<double>(segmentIndex);
			
			Point startPos = nodes[currentNode] + (alpha * deltaDistance * normalizedDir);
			Point endPos = nodes[currentNode] + ((alpha + 1.f) * deltaDistance * normalizedDir);
			Point pos = (startPos + endPos) / 2.f;
			
			double startTime = currentTime + (alpha * deltaTime);
			startTime -= (timing.getUncertaintyForReservation(startTime, Direction::BEHIND) + reservationTimeMarginBehind);
			
			double endTime = currentTime + ((alpha + 1.f) * deltaTime);
			endTime += (timing.getUncertaintyForReservation(endTime, Direction::AHEAD) + reservationTimeMarginAhead);

			reservations.emplace_back(pos, Point(deltaTime + reservationSize, reservationSize), currentRotation, startTime, endTime, ownerId);
		}

		currentTime += drivingTimes.at(i);
	}
	
	// path computation is responsible for checking that this area is free for the specified time
	if(targetReservationTime > 0) {		
		double startTime = currentTime - reservationTimeMarginBehind;
		double endTime = currentTime + targetReservationTime + reservationTimeMarginAhead;
		
		// Block approach space		
		double offset = APPROACH_DISTANCE + 0.1f; // + distanceWhenApproached
		double lengthMargin = 0.275f;
		double widthMargin = 0.05f;
		Point pos = nodes.back() + Math::getVectorFromOrientation(end.o) * offset;		
		double length = (ROBOT_RADIUS + offset + lengthMargin) * 2.f;
		double width = (ROBOT_RADIUS + widthMargin) * 2.f;
		reservations.emplace_back(pos, Point(length, width), Math::toDeg(end.o), startTime, endTime, ownerId);

		// Block neighbour trays space
		widthMargin = 0.235f; // Cover neighbouring trays too
		pos = nodes.back() + Math::getVectorFromOrientation(end.o) * offset;
		length = (ROBOT_RADIUS) * 2.f;
		width = (ROBOT_RADIUS + widthMargin) * 2.f;
		reservations.emplace_back(pos, Point(length, width), Math::toDeg(end.o), startTime, endTime, ownerId);
	}

	return reservations;
}

const std::vector<Point>& Path::getNodes() const {
	ROS_ASSERT(isValidPath);
	return nodes;
}

const std::vector<double>& Path::getWaitTimes() const {
	ROS_ASSERT(isValidPath);
	return waitTimes;
}

const std::vector<double>& Path::getDepartureTimes() const {
	ROS_ASSERT(isValidPath);
	return departureTimes;
}

double Path::getDistance() const {
	ROS_ASSERT(isValidPath);
	return distance;
}

double Path::getDuration() const {
	ROS_ASSERT(isValidPath);
	return duration;
}

double Path::getBatteryConsumption() const {
	ROS_ASSERT(isValidPath);
	return batteryConsumption;
}

visualization_msgs::Marker Path::getVisualizationMsgLines(std_msgs::ColorRGBA color) {
	ROS_ASSERT(isValidPath);
	visualization_msgs::Marker msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.ns = "PathLines";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.orientation.w = 1.0;

	msg.id = 1;
	msg.type = visualization_msgs::Marker::LINE_STRIP;
	msg.scale.x = 0.1;

	msg.color = color;
	msg.color.a = 0.6f;
	
	for(const Point& point : nodes) {
		geometry_msgs::Point p;
		p.x = point.x;
		p.y = point.y;
		p.z = 0.f;
		msg.points.push_back(p);
	}

	return msg;
}

double Path::getStartTimeOffset() const {
	ROS_ASSERT(isValidPath);
	return startTimeOffset;
}

RobotHardwareProfile* Path::getRobotHardwareProfile() const {
	ROS_ASSERT(isValidPath);
	return hardwareProfile;
}

OrientedPoint Path::getStart() {
	ROS_ASSERT(isValidPath);
	return start;
}

OrientedPoint Path::getEnd() {
	ROS_ASSERT(isValidPath);
	return end;
}

bool Path::isValid() const {
	return isValidPath;
}

