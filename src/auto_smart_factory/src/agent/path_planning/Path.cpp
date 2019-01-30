#include <utility>
#include <cmath>

#include "Math.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "agent/path_planning/Path.h"

Path::Path(double startTimeOffset, std::vector<Point> nodes_, std::vector<double> waitTimes_, RobotHardwareProfile* hardwareProfile) :
		startTimeOffset(startTimeOffset),
		nodes(std::move(nodes_)),
		waitTimes(std::move(waitTimes_)),
		hardwareProfile(hardwareProfile)
{
	if(nodes.size() != waitTimes.size()) {
		ROS_ERROR("nodes.size() != waitTimes.size()");
	}

	// Turning time is considered as part of the following line segment.
	// Therefore, a line segment driving time = Time to turn to target Point + driving time to target point 

	if(!nodes.empty()) {
		duration += waitTimes.at(0);
		batteryConsumption += hardwareProfile->getIdleBatteryConsumption(waitTimes.at(0));
		departureTimes.push_back(startTimeOffset + waitTimes.at(0));

		if(nodes.size() > 1) {
			for(unsigned long i = 1; i < nodes.size(); i++) { // last waitTime is not used
				float currentDistance = Math::getDistance(nodes.at(i - 1), nodes.at(i));

				// Don't compute angle difference for last node
				float angleDifference = 0;
				if(i < nodes.size() - 1) {
					angleDifference = Math::getAngleDifferenceInDegree(Math::getRotation(nodes[i] - nodes[i - 1]), Math::getRotation(nodes[i + 1] - nodes[i]));
				}

				distance += currentDistance;
				duration += hardwareProfile->getDrivingDuration(currentDistance);
				duration += waitTimes.at(i);

				batteryConsumption += hardwareProfile->getIdleBatteryConsumption(waitTimes.at(i));
				batteryConsumption += hardwareProfile->getDrivingBatteryConsumption(hardwareProfile->getDrivingDuration(currentDistance));
				batteryConsumption += hardwareProfile->getDrivingBatteryConsumption(hardwareProfile->getTurningDuration(angleDifference));

				// Departure times does not include turningTime as this is considered part of the next segment
				departureTimes.push_back(startTimeOffset + duration);
				duration += hardwareProfile->getTurningDuration(angleDifference);
			}
		}
	}
}

const std::vector<Rectangle> Path::generateReservations(int ownerId) const {
	std::vector<Rectangle> reservations;

	float reservationSize = ROBOT_DIAMETER * 2.0f;

	float currentTime = 0;
	for(unsigned int i = 0; i < nodes.size() - 1; i++) {
		float currentLength = Math::getDistance(nodes[i], nodes[i + 1]);
		Point currentDir = (nodes[i + 1] - nodes[i]) * 1.f * (1.f/currentLength);
		float currentRotation = Math::getRotation(currentDir);

		// Waiting time
		if(waitTimes.at(i) > 0) {
			reservations.emplace_back(nodes[i], Point(reservationSize, reservationSize), currentRotation, currentTime, currentTime + waitTimes[i], ownerId);
			currentTime += waitTimes[i];
		}

		// Line segment
		auto segmentCount = static_cast<unsigned int>(std::ceil(currentLength / maxReservationLength));
		float segmentLength = currentLength / static_cast<float>(segmentCount);

		for(unsigned int segment = 0; segment < segmentCount; segment++) {
			Point startPos = nodes[i] + (segment * segmentLength * currentDir);
			Point endPos = nodes[i] + ((segment + 1) * segmentLength * currentDir);

			Point pos = (startPos + endPos) / 2.f;
			double startTime = startTimeOffset + currentTime + (segment * segmentLength) - reservationSize / 2.f;
			double endTime = startTimeOffset + currentTime + ((segment + 1) * segmentLength) + reservationSize / 2.f;

			reservations.emplace_back(pos, Point(segmentLength + reservationSize, reservationSize), currentRotation, startTime, endTime, ownerId);
		}

		currentTime += currentLength;
	}

	return reservations;
}

const std::vector<Point>& Path::getNodes() const {
	return nodes;
}

const std::vector<double>& Path::getWaitTimes() const {
	return waitTimes;
}

const std::vector<double>& Path::getDepartureTimes() const {
	return departureTimes;
}

float Path::getDistance() const {
	return distance;
}

double Path::getDuration() const {
	return duration;
}

float Path::getBatteryConsumption() const {
	return batteryConsumption;
}

visualization_msgs::Marker Path::getVisualizationMsgPoints() {
	visualization_msgs::Marker msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.ns = "PathPoints";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.orientation.w = 1.0;

	msg.id = 0;
	msg.type = visualization_msgs::Marker::POINTS;
	msg.scale.x = 0.1;
	msg.scale.y = 0.1;

	msg.color.g = 1.0f;
	msg.color.r = 0.5f;
	msg.color.a = 1.0;

	for(const Point& point : nodes) {
		geometry_msgs::Point p;
		p.x = point.x;
		p.y = point.y;
		p.z = 0.f;
		msg.points.push_back(p);
	}
	
	return msg;
}

visualization_msgs::Marker Path::getVisualizationMsgLines() {

	visualization_msgs::Marker msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.ns = "PathLines";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.orientation.w = 1.0;

	msg.id = 1;
	msg.type = visualization_msgs::Marker::LINE_STRIP;
	msg.scale.x = 0.1;

	// Set colors
	ros::NodeHandle nh("~");
	double color_r;
	double color_g;
	double color_b;
	if(!nh.getParam("color_r", color_r)) {
		color_r = 30;
	}
	
	if(!nh.getParam("color_g", color_g)) {
		color_g = 200;
	}

	if(!nh.getParam("color_b", color_b)) {
		color_b = 40;
	}
	msg.color.a = 0.6f;
	msg.color.r = color_r / 255;
	msg.color.g = color_g / 255;
	msg.color.b = color_b / 255;


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
	return startTimeOffset;
}

RobotHardwareProfile* Path::getRobotHardwareProfile() const {
	return hardwareProfile;
}
