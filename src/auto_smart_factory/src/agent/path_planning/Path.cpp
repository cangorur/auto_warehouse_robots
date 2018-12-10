#include <utility>

#include <agent/path_planning/Path.h>
#include <include/agent/path_planning/Path.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

Path::Path(std::vector<Point> points_)
		:
		points(std::move(points_)) {

	for(Point& point : points) {
		// TODO getDistance...
	}
}

const std::vector<Point>& Path::getPoints() const {
	return points;
}

float Path::getLength() const {
	return length;
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

	for(const Point& point : points) {
		geometry_msgs::Point p;
		p.x = point.x;
		p.y = point.y;
		p.z = 1.f;
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

	msg.color.g = 1.0f;
	msg.color.a = 1.0;

	for(const Point& point : points) {
		geometry_msgs::Point p;
		p.x = point.x;
		p.y = point.y;
		p.z = 1.f;
		msg.points.push_back(p);
	}

	return msg;
}
