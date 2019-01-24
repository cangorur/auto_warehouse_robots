#include <cmath>

#include <utility>

#include "ros/ros.h"

#include "agent/Agent.h"
#include <visualization_msgs/Marker.h>

#include "agent/MotionPlanner.h"

#include <ros/console.h>

#include <cmath>

MotionPlanner::MotionPlanner(Agent* a, auto_smart_factory::RobotConfiguration robot_config, ros::Publisher* motion_pub) :
	pathObject(Path({}))
{
	robotConfig = std::move(robot_config);
	
	minDrivingSpeed = robotConfig.min_linear_vel;
	maxDrivingSpeed = robotConfig.max_linear_vel;
	maxTurningSpeed = robotConfig.max_angular_vel;
	
	motionPub = motion_pub;
	agent = a;
	agentID = agent->getAgentID();

	ros::NodeHandle n;
	pathPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	steerPid = new PidController(0.0, 1.8, 0.0, 4.0);
}

MotionPlanner::~MotionPlanner() {
	delete steerPid;
};

void MotionPlanner::update(geometry_msgs::Point position, double orientation) {
	pos.update(position.x, position.y, orientation, ros::Time::now());

	if (mode == Mode::FINISHED || mode == Mode::STOP) {
		publishVelocity(0.0, 0.0);
		return;
	}

	if (mode == Mode::ALIGN) {
		if (alignDirection == -1.0)
			turnTowards(alignTarget);
		else
			turnTowards(alignDirection);

		return;
	}

	if (waypointReached(&pos)) {
		if (!isCurrentPointLastPoint())	{
			advanceToNextPathPoint();
		} else {
			mode = Mode::FINISHED;
			publishVelocity(0.0, 0.0);
			return;
		}
	}

	if (mode == Mode::TURN || std::abs(getRotationToTarget(pos, currentTarget)) >= turnThreshold) {
		mode = Mode::TURN;
		turnTowards(currentTarget);
		return;
	}

	double cte = Math::getDistanceToLine(previousTarget, currentTarget, Point(pos.x, pos.y)) * Math::getDirectionToLineSegment(previousTarget, currentTarget, Point(position.x, position.y));
	double angularVelocity = steerPid->calculate(cte, ros::Time::now().toSec());
	angularVelocity = std::min(std::max(angularVelocity, (double) -maxTurningSpeed), (double) maxTurningSpeed);

	double linearVelocity = maxDrivingSpeed - std::min((std::exp(cte*cte)-1), (double) maxDrivingSpeed-minDrivingSpeed);

	if (isCurrentPointLastPoint() && Math::getDistance(Point(pos.x, pos.y), currentTarget) < 0.4f) {
		linearVelocity = 0.1;
	}

	publishVelocity(linearVelocity, angularVelocity);

	/* Debug print
	if (agentID.compare("robot_2") == 0) {
		printf("[MP %s] cte: %.4f | speed: %.4f | steer: %.4f\n", agentID.c_str(), cte, linearVelocity, angularVelocity);
	}
	*/
}

void MotionPlanner::turnTowards(Point target) {
	double rotation = getRotationToTarget(pos, target);
	if(std::abs(rotation) <= 0.1f) {
		mode = Mode::PID;
		return;
	}
	publishVelocity(0, Math::clamp(std::abs(rotation), 0, maxTurningSpeed) * (rotation < 0.f ? -1.f : 1.f));
}

void MotionPlanner::turnTowards(double direction) {
	double rotation = static_cast<double>(Math::getAngleDifferenceInRad(pos.o, direction));
	ROS_FATAL("MP: rotation: %.4f | robot: %.4f | target: %.4f", rotation, pos.o, direction);
	if(std::abs(rotation) <= 0.1f) {
		mode = Mode::FINISHED;
		return;
	}
	publishVelocity(0, Math::clamp(std::abs(rotation), 0, maxTurningSpeed) * (rotation < 0.f ? -1.f : 1.f));
}

void MotionPlanner::alignTowards(Point target) {
	mode = Mode::ALIGN;
	alignTarget = target;
	alignDirection = -1.0;
}

void MotionPlanner::alignTowards(double direction) {
	mode = Mode::ALIGN;
	alignDirection = direction;
}

void MotionPlanner::newPath(geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> new_path, geometry_msgs::Point end_direction_point, bool drive_backwards) {
	std::vector<Point> points;
	
	for(geometry_msgs::Point p : new_path) {
		points.emplace_back(p.x, p.y);
	}
	
	pathObject = Path(points);
	mode = Mode::READY;
	currentTarget = pathObject.getPoints().front();
	currentTargetIndex = 0;
	previousTarget = Point(start_position.x, start_position.y);
	this->newPath(Path(points));
}

void MotionPlanner::newPath(Path path) {
	pathObject = path;
	
	if(path.getLength() > 0) {
		currentTarget = pathObject.getPoints().front();
		currentTargetIndex = 0;
		mode = Mode::READY;
		agent->getVisualisationPublisher()->publish(pathObject.getVisualizationMsgPoints());
		agent->getVisualisationPublisher()->publish(pathObject.getVisualizationMsgLines());
	} else {
		ROS_INFO("[MotionPlanner - %s]: Got path with length 0", agentID.c_str());
		mode = Mode::FINISHED;
	}	
}

MotionPlanner::Mode MotionPlanner::getMode() {
	return mode;
}

void MotionPlanner::start() {
	mode = Mode::READY;
}

void MotionPlanner::stop() {
	mode = Mode::STOP;
	publishVelocity(0.0, 0.0);
}

bool MotionPlanner::isDone() {
	return (mode == Mode::FINISHED);
}

bool MotionPlanner::hasPath() {
	return pathObject.getLength() > 0;
}

void MotionPlanner::publishVelocity(double speed, double angle) {
	geometry_msgs::Twist msg;

	msg.linear.x = speed;
	msg.angular.z = angle;

	motionPub->publish(msg);
}

bool MotionPlanner::waypointReached(Position *current) {
	if (!isCurrentPointLastPoint())	{
		return (Math::getDistance(Point(current->x, current->y), currentTarget) <= distToReachPoint);
	} else {
		return (Math::getDistance(Point(current->x, current->y), currentTarget) <= distToReachFinalPoint);
	}
}

bool MotionPlanner::driveCurrentPath(Position currentPosition) {
	geometry_msgs::Twist motion;

	float distToTarget = Math::getDistance(Point(currentPosition.x, currentPosition.y), currentTarget);
	float rotationToTarget = getRotationToTarget(currentPosition, currentTarget);
	
	float rotationSign = rotationToTarget < 0.f ? -1.f : 1.f;
	rotationToTarget = std::fabs(rotationToTarget);
	
	if(rotationToTarget < allowedRotationDifference) {
		rotationToTarget = 0;
	}
	
	// has reached next point?
	if(distToTarget <= distToReachPoint && !isCurrentPointLastPoint()) {
		// Rerun with new point
		advanceToNextPathPoint();
		return driveCurrentPath(currentPosition);
	} else if(distToTarget <= distToReachFinalPoint && isCurrentPointLastPoint()) {
		// Todo rotate till final rotation is guaranteed
		stop();
		motionPub->publish(motion);
		mode = Mode::FINISHED;
		return true;
	}
	
	// Totally wrong direction
	if(rotationToTarget >= maxRotationDifference) {
		motion.angular.z = Math::clamp(rotationToTarget, 0, maxTurningSpeed) * rotationSign;
		motion.linear.x = 0;
	} else {
		float rotationAlpha = Math::clamp(rotationToTarget/maxRotationDifference, 0, 1);
		motion.angular.z = Math::lerp(minTurningSpeed, maxTurningSpeed, rotationAlpha) * rotationSign;
		
		float speedAlphaThroughRotation = 1.f - rotationAlpha;
		if(speedAlphaThroughRotation < 0.9f) {
			speedAlphaThroughRotation /= 3.f;
		}
		
		float speedAlphaThroughDist = 1.f;
		if(distToTarget <= distToSlowDown) {
			speedAlphaThroughDist = Math::clamp((distToTarget - distToReachPoint) / (distToSlowDown - distToReachPoint), 0, 1.f);
		}
		
		float speedAlpha = std::min(speedAlphaThroughDist, speedAlphaThroughRotation);
		
		motion.linear.x = Math::lerp(minDrivingSpeed, maxDrivingSpeed, speedAlpha);
	}
	
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

float MotionPlanner::getRotationFromOrientationDifference(double orientation) {
	return static_cast<float>(orientation * 180.f);
}

void MotionPlanner::advanceToNextPathPoint() {
	previousTarget = pathObject.getPoints().at(static_cast<unsigned long>(currentTargetIndex));
	currentTargetIndex++;
	currentTarget = pathObject.getPoints().at(static_cast<unsigned long>(currentTargetIndex));
}

bool MotionPlanner::isCurrentPointLastPoint() {
	return currentTargetIndex == pathObject.getPoints().size() - 1;
}

bool MotionPlanner::isDrivingBackwards() {
	return false;
}

double MotionPlanner::getRotationToTarget(Position currentPosition, Point targetPosition) {
	double direction = std::atan2(targetPosition.y - currentPosition.y, targetPosition.x - currentPosition.x);

	return static_cast<double>(Math::getAngleDifferenceInRad(currentPosition.o, direction));
}

visualization_msgs::Marker MotionPlanner::getVisualizationMsgPoints() {
	return pathObject.getVisualizationMsgPoints();
}

visualization_msgs::Marker MotionPlanner::getVisualizationMsgLines() {
	return pathObject.getVisualizationMsgLines();
}

