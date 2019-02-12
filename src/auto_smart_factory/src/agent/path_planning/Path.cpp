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
	duration = 0;
	distance = 0;
	batteryConsumption = 0;

	for(unsigned long i = 0; i < nodes.size() - 1; i++) { // last waitTime is not used
		double currentDistance = Math::getDistance(nodes.at(i), nodes.at(i + 1));

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
			drivingTime = hardwareProfile->getDrivingDuration(currentDistance);
		} else {
			onSpotTime = waitTimes.at(i);
			drivingTime = hardwareProfile->getDrivingDuration(currentDistance) + turningDuration;
		}
		
		// Distance
		distance += currentDistance;
		
		// Duration: Driving is part of the next segment, add after departureTime
		duration += onSpotTime;
		departureTimes.push_back(startTimeOffset + duration - 0.5f);
		duration += drivingTime;
		
		batteryConsumption += hardwareProfile->getIdleBatteryConsumption(onSpotTime) + hardwareProfile->getDrivingBatteryConsumption(currentDistance);
		
		onSpotTimes.push_back(onSpotTime);
		drivingTimes.push_back(drivingTime);
	}
	
	// TargetPoint - getTurningTime also works with direction reversed. nodes.size is guaranteed to be >= 2
	double finalTurningTime = timing.getTurningTime(Math::toDeg(end.o), nodes.at(nodes.size() - 2), nodes.back());
	duration += finalTurningTime;
	batteryConsumption += hardwareProfile->getIdleBatteryConsumption(finalTurningTime);

}

Path::Path() : 
	isValidPath(false)
{
}

const std::vector<Rectangle> Path::generateReservations(int ownerId) const {
	std::vector<Rectangle> reservations;
	Point waitingReservationSize = Point(getReservationSize(), getReservationSize()) * 0.98f;
	double maxMergeDistance = 0.45f;

	double currentTime = startTimeOffset;
	for(unsigned int i = 0; i < nodes.size() - 1; i++) {
		// OnSpotTime
		if(onSpotTimes.at(i) > 0 || i == 0) {
			double startTime = currentTime - timing.getReservationUncertainty(currentTime, Direction::BEHIND) - reservationTimeMarginBehind;
			double endTime = currentTime + onSpotTimes[i] + timing.getReservationUncertainty(currentTime + onSpotTimes[i], Direction::AHEAD) + reservationTimeMarginAhead;

			reservations.emplace_back(nodes[i], waitingReservationSize, 0, startTime, endTime, ownerId);
			currentTime += onSpotTimes[i];
		}

		// Try to merge points to curve
		unsigned int j = i;
		double curveSegmentsDrivingTime = 0;
		std::vector<Point> curvePoints;
		while(j < i + 5 && j < nodes.size() - 1) {
			double distance = Math::getDistance(nodes[j], nodes[j + 1]);
			
			if(distance <= maxMergeDistance && onSpotTimes[j] == 0) {
				curvePoints.push_back(nodes[j]);
				curveSegmentsDrivingTime += drivingTimes[j];
				j++;
			} else {
				break;
			}
		}
		
		if(curvePoints.size() > 1) {
			generateReservationsForCurvePoints(reservations, curvePoints, currentTime, curveSegmentsDrivingTime, ownerId);
			currentTime += curveSegmentsDrivingTime;
			i = j - 1; // -1 because for-loop ++
			continue;
		}

		generateReservationsForSegment(reservations, nodes[i], nodes[i + 1], currentTime, drivingTimes.at(i), ownerId);
		currentTime += drivingTimes.at(i);
	}
	
	if(targetReservationTime > 0) {		
		generateReservationForTray(reservations, currentTime, ownerId);
	}

	return reservations;
}

void Path::generateReservationsForSegment(std::vector<Rectangle>& reservations, Point startPoint, Point endPoint, double timeAtStartPoint, double deltaTime, int ownerId) const {
	double distance = Math::getDistance(startPoint, endPoint);
	Point normalizedDir = (endPoint - startPoint) * (1.f/distance);
	double rotation = Math::getRotationInDeg(normalizedDir);
	
	auto segmentCount = static_cast<unsigned int>(std::ceil(deltaTime / maxDrivingReservationDuration));
	double deltaDuration = deltaTime / static_cast<double>(segmentCount);
	double deltaDistance = distance / static_cast<double>(segmentCount);

	for(unsigned int segmentIndex = 0; segmentIndex < segmentCount; segmentIndex++) {
		auto alpha = static_cast<double>(segmentIndex);

		Point startPos = startPoint + (alpha * deltaDistance * normalizedDir);
		Point endPos = startPoint + ((alpha + 1.f) * deltaDistance * normalizedDir);
		Point pos = (startPos + endPos) / 2.f;

		double startTime = timeAtStartPoint + (alpha * deltaDuration);
		startTime -= (timing.getReservationUncertainty(startTime, Direction::BEHIND) + reservationTimeMarginBehind);

		double endTime = timeAtStartPoint + ((alpha + 1.f) * deltaDuration);
		endTime += (timing.getReservationUncertainty(endTime, Direction::AHEAD) + reservationTimeMarginAhead);

		reservations.emplace_back(pos, Point(deltaDistance + getReservationSize(), getReservationSize()), rotation, startTime, endTime, ownerId);
	}
}

void Path::generateReservationsForCurvePoints(std::vector<Rectangle>& reservations, std::vector<Point> points, double timeAtStartPoint, double deltaTime, int ownerId) const {
	// TODO maybe make reservation covering all points larger
	double distance = Math::getDistance(points.front(), points.back());
	Point normalizedDir = (points.back() - points.front()) * (1.f/distance);
	double rotation = Math::getRotationInDeg(normalizedDir);

	auto segmentCount = static_cast<unsigned int>(std::ceil(deltaTime / maxDrivingReservationDuration));
	double deltaDuration = deltaTime / static_cast<double>(segmentCount);
	double deltaDistance = distance / static_cast<double>(segmentCount);

	for(unsigned int segmentIndex = 0; segmentIndex < segmentCount; segmentIndex++) {
		auto alpha = static_cast<double>(segmentIndex);

		Point startPos = points.front() + (alpha * deltaDistance * normalizedDir);
		Point endPos = points.front() + ((alpha + 1.f) * deltaDistance * normalizedDir);
		Point pos = (startPos + endPos) / 2.f;

		double startTime = timeAtStartPoint + (alpha * deltaDuration);
		startTime -= (timing.getReservationUncertainty(startTime, Direction::BEHIND) + reservationTimeMarginBehind);

		double endTime = timeAtStartPoint + ((alpha + 1.f) * deltaDuration);
		endTime += (timing.getReservationUncertainty(endTime, Direction::AHEAD) + reservationTimeMarginAhead);

		reservations.emplace_back(pos, Point(deltaDistance + getReservationSize(), getReservationSize()), rotation, startTime, endTime, ownerId);
	}
}

void Path::generateReservationForTray(std::vector<Rectangle>& reservations, double timeAtStartPoint, int ownerId) const {
	double startTime = timeAtStartPoint - reservationTimeMarginBehind;
	double endTime = timeAtStartPoint + targetReservationTime + reservationTimeMarginAhead;

	// Block approach space		
	double offset = APPROACH_DISTANCE + 0.1f; // + distanceWhenApproached
	double lengthMargin = 0.275f;
	double widthMargin = 0.1f;
	Point pos = Point(end.x, end.y) + Math::getVectorFromOrientation(end.o) * offset;
	double length = (ROBOT_RADIUS + offset + lengthMargin) * 2.f;
	double width = (ROBOT_RADIUS + widthMargin) * 2.f;
	
	reservations.emplace_back(pos, Point(length, width), Math::toDeg(end.o), startTime, endTime, ownerId);

	// Block neighbour trays space
	widthMargin = 0.235f; // Cover neighbouring trays too
	length = (ROBOT_RADIUS) * 2.f;
	width = (ROBOT_RADIUS + widthMargin) * 2.f;
	
	reservations.emplace_back(pos, Point(length, width), Math::toDeg(end.o), startTime, endTime, ownerId);
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
	msg.scale.x = 0.06f;

	msg.color = color;
	msg.color.a = 0.75f;
	
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

double Path::getReservationSize() const {
	return ROBOT_RADIUS * 2.0f;
}

