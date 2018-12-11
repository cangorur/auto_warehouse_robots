#include <cmath>

#include <utility>

#include "agent/Agent.h"
#include <visualization_msgs/Marker.h>
#include <include/agent/MotionPlanner.h>

#include "agent/MotionPlanner.h"
#include "Math.h"
#include "agent/path_planning/Point.h"

#include <ros/console.h>


MotionPlanner::MotionPlanner(Agent* a, auto_smart_factory::RobotConfiguration robot_config, ros::Publisher* motion_pub) {
	robotConfig = std::move(robot_config);
	
	minDrivingSpeed = robotConfig.min_linear_vel;
	maxDrivingSpeed = robotConfig.max_linear_vel;
	maxTurningSpeed = robotConfig.max_angular_vel;
	
	motionPub = motion_pub;
	agent = a;
	agentID = agent->getAgentID();

	ros::NodeHandle n;
	pathPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

MotionPlanner::~MotionPlanner() = default;

void MotionPlanner::update(geometry_msgs::Point position, double orientation) {
	driveCurrentPath(Point(position.x, position.y), orientation);
}

void MotionPlanner::newPath(geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> new_path, geometry_msgs::Point end_direction_point, bool drive_backwards) {
	std::vector<Point> points;
	
	for(geometry_msgs::Point p : new_path) {
		points.emplace_back(p.x, p.y);
	}
	
	pathObject = Path(points);
	hasFinishedCurrentPath = false;
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

void MotionPlanner::start() {
	standStill = false;
}

void MotionPlanner::stop() {
	standStill = true;
	geometry_msgs::Twist motion;
	motionPub->publish(motion);
}

bool MotionPlanner::isDone() {
	return hasFinishedCurrentPath;
}

bool MotionPlanner::hasPath() {
	return pathObject.getLength() > 0;
}

bool MotionPlanner::driveCurrentPath(Point currentPosition, double orientation) {
	geometry_msgs::Twist motion;

	float distToTarget = Math::getDistance(currentPosition, currentTarget);

	// WTF
	orientation = orientation / (PI / 2.0);
	float desiredRotation = Math::getRotation(currentTarget - currentPosition);
	float currentRotation = getRotationFromOrientation(orientation);
	float rotationToTarget = Math::getAngleDifference(currentRotation, desiredRotation);

	//float rotationToTarget = getRotationToTarget(currentPosition, currentTarget, orientation);
	
	float rotationSign = rotationToTarget < 0.f ? -1.f : 1.f;
	rotationToTarget = std::fabs(rotationToTarget);
	
	if(rotationToTarget < allowedRotationDifference) {
		//rotationToTarget = 0;
	}
	
	// has reached next point?
	if(distToTarget <= distToReachPoint && !isCurrentPointLastPoint()) {
		// Rerun with new point
		advanceToNextPathPoint();
		return driveCurrentPath(currentPosition, orientation);
	} else if(distToTarget <= distToReachFinalPoint && isCurrentPointLastPoint()) {
		// Todo rotate till final rotation is guaranteed
		stop();
		motionPub->publish(motion);
		hasFinishedCurrentPath = true;
		return true;
	}
	
	// Totally wrong direction
	/*if(rotationToTarget >= maxRotationDifference) {
		motion.angular.z = Math::clamp(rotationToTarget, 0, maxTurningSpeed) * rotationSign;
	} else {
		float rotationAlpha = Math::clamp(rotationToTarget/maxRotationDifference, 0, 1);
		motion.angular.z = Math::lerp(minTurningSpeed, maxTurningSpeed, rotationAlpha) * rotationSign;
		
		float forwardSpeedAlpha = 1.f - rotationAlpha;
		motion.linear.x = Math::lerp(minDrivingSpeed, maxDrivingSpeed, forwardSpeedAlpha);		
	}*/
	
	motion.angular.z = rotationSign * Math::clamp(rotationToTarget, minTurningSpeed, maxTurningSpeed);
	
	motionPub->publish(motion);
	return false;
}

/* Returns the angle of the given orientation.
	 * @param orientation: orientation e.g
	 *       0.5
	 *        ||
	 *  1/-1--  --0
	 *        ||
	 *      -0.5
	  @returns an orientation angle e.g.
	 *      270
	 *      ||
	 * 180--  --0/360
 	 *      ||
	 *     90
	 *     */
float MotionPlanner::getRotationFromOrientation(double orientation) {
	if(orientation > 1.f) {
		orientation = -(2.f - orientation);
	} else if(orientation < -1.f) {
		orientation = 2.f + orientation;
	}
	
	if(orientation < 0) {
		return static_cast<float>(std::fabs(orientation) * 180.f);
	} else {
		return static_cast<float>(360.f - orientation * 180.f);
	}
}
/*
double MotionPlanner::getDirection(double p_x, double p_y, double t_x, double t_y) {
	return (std::atan2(t_y - p_y, t_x - p_x)) / PI;
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
}*/

void MotionPlanner::advanceToNextPathPoint() {
	currentTargetIndex++;
	currentTarget = pathObject.getPoints().at(static_cast<unsigned long>(currentTargetIndex));
}

bool MotionPlanner::isCurrentPointLastPoint() {
	return currentTargetIndex == pathObject.getPoints().size() - 1;
}

bool MotionPlanner::isDrivingBackwards() {
	return false;
}

float MotionPlanner::getRotationToTarget(Point currentPosition, Point targetPosition, double orientation) {
	double direction = (std::atan2(targetPosition.y - currentPosition.y, targetPosition.x - currentPosition.x)) / PI;

	double diff = 0;
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
	
	return getRotationFromOrientation(diff);	
}

