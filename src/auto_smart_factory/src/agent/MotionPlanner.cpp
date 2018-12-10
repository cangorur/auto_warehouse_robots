#include <utility>

#include "agent/Agent.h"
#include <visualization_msgs/Marker.h>
#include "agent/MotionPlanner.h"
#include "Math.h"
#include "agent/path_planning/Point.h"


MotionPlanner::MotionPlanner(Agent* a, auto_smart_factory::RobotConfiguration robot_config, ros::Publisher* motion_pub) {
	robotConfig = std::move(robot_config);
	drivingMinSpeed = robotConfig.min_linear_vel;
	drivingMaxSpeed = robotConfig.max_linear_vel;
	turningMaxSpeed = robotConfig.max_angular_vel;
	motionPub = motion_pub;
	agent = a;
	agentID = agent->getAgentID();
	std::stringstream ss;
	ss << "/motion_planner_" << agentID << "/activate_tests";
	test_enable_paramStr = ss.str();
	ros::param::set(test_enable_paramStr, tests_enabled); // false by default

	ros::NodeHandle n;
	pathPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

MotionPlanner::~MotionPlanner() = default;

void MotionPlanner::update(geometry_msgs::Point position, double orientation) {
	driveCurrentPath(Point(position.x, position.y), orientation);
}

void MotionPlanner::newPath(geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> new_path, geometry_msgs::Point end_direction_point, bool drive_backwards) {
	std::vector<Point> points;
	
	test(1);
	
	for(geometry_msgs::Point p : new_path) {
		points.emplace_back(p.x, p.y);
	}
	
	pathObject = Path(points); 
	done = false;
	currentTarget = pathObject.getPoints().front();
	currentTargetIndex = 0;

	pathPub.publish(pathObject.getVisualizationMsgPoints());
	pathPub.publish(pathObject.getVisualizationMsgLines());
}

void MotionPlanner::enable(bool enable) {
	this->enabled = enable;
}

bool MotionPlanner::isEnabled() {
	return this->enabled;
}

bool MotionPlanner::isTestsEnabled() { // adjusted through ros param
	return this->tests_enabled;
}

void MotionPlanner::start() {
	standStill = false;
}

void MotionPlanner::stop() {
	standStill = true;
	geometry_msgs::Twist motion;
	motionPub->publish(motion);
}

bool MotionPlanner::isDone() {
	return done;
}

bool MotionPlanner::hasPath() {
	//return path.size() > 0;
	return pathObject.getLength() > 0;
}

bool MotionPlanner::isDrivingBackwards() {
	return driveBackwards;
}

bool MotionPlanner::driveCurrentPath(Point currentPosition, double orientation) {
	geometry_msgs::Twist motion;
	
	float distToTarget = Math::getDistance(currentPosition, currentTarget);
	//float desiredRotation = Math::getRotation(currentTarget - currentPosition);
	motion.linear.x = 1; // m/s
	motion.angular.z = 1;   //rad/s  // 1rad ~= 60°
	
	motionPub->publish(motion);
	
	
	

	/*{
		motionPub->publish(motion);
		return false;
	} else {
		stop();
		return true;
	}*/
}

double MotionPlanner::getDirection(double p_x, double p_y, double t_x, double t_y) {
	return (std::atan2(t_y - p_y, t_x - p_x)) / PI;
}

double MotionPlanner::getDistance(double p1_x, double p1_y, double p2_x, double p2_y) {
	return sqrt(pow(p1_x - p2_x, 2.0) + pow(p1_y - p2_y, 2.0));
}

double MotionPlanner::getOrientationDiff(double orientation, double direction) {
	double diff;
	if(orientation >= 0 && direction >= 0) {
		diff = direction - orientation;
	} else if(orientation < 0 && direction < 0) {
		diff = direction - orientation;
	} else if(orientation >= 0 && direction < 0) {
		if(orientation - direction <= 1) {
			diff = direction - orientation;
		} else {
			diff = (1 - orientation) + (1 + direction);
		}
	} else if(orientation < 0 && direction >= 0) {
		if(direction - orientation <= 1) {
			diff = direction - orientation;
		} else {
			diff = direction - orientation - 2;
		}
	}
	return diff;
}

double MotionPlanner::getAngle(double orientation) {
	return (orientation >= 0) ? 180 * orientation : 360 + 180 * orientation;
}

void MotionPlanner::emgRetreat(float stop_interval, float retreat_interval) {
	geometry_msgs::Twist motion;
	motion.linear.x = -0.1;
	motion.linear.y = 0.0;
	motion.angular.x = 0.0;
	motion.angular.y = 0.0;
	motion.angular.z = 0.0;
	stop();
	ros::Duration(1 * stop_interval).sleep();
	motionPub->publish(motion);
	ROS_INFO("Retreating");
	ros::Duration(1 * retreat_interval).sleep();
	stop();
	start();
}

float MotionPlanner::clamp(float value, float min, float max) const {
	return std::max(min, std::min(max, value));
}

float MotionPlanner::lerp(float start, float end, float alpha) const {
	return start + (end-start) * alpha;
}

float MotionPlanner::getAngleFromOrientation(double orientation) {
	double angleInverted = getAngle(orientation);
	
	while(angleInverted >= 360.f) {
		angleInverted -= 360.f;
	}

	return 360.f - angleInverted;
}

