#include <cmath>

#include <utility>

#include "agent/Agent.h"
#include <visualization_msgs/Marker.h>

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

	pidInit(0.2, 0.4, 1.0, 2.0);
}

MotionPlanner::~MotionPlanner() {
	delete start;
	delete last;
};

void MotionPlanner::update(geometry_msgs::Point position, double orientation) {
	//driveCurrentPath(Point(position.x, position.y), orientation);
	Position *pos = new Position(position.x, position.y, orientation, ros::Time::now());
	if(waypointReached()) {
		if (!isCurrentPointLastPoint())	{
			ROS_WARN("[MotionPlanner] currentTargetIndex: %d | PathObjectSize: %d", currentTargetIndex, pathObject.getPoints().size() - 1);
			advanceToNextPathPoint();
			pidReset();
			pidSetTarget(currentTarget, Position(position.x, position.y, orientation, ros::Time::now()));
		}
	} else {
		pidUpdate(pos);
	}
	delete pos;
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

	atTarget = true;
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

void MotionPlanner::pidInit(double posTolerance, double angleTolerance, double maxSpeed, double maxAngleSpeed) {
	this->posTolerance = posTolerance;
	this->angleTolerance = angleTolerance;
	this->maxSpeed = maxSpeed;
	this->maxAngleSpeed = maxAngleSpeed;
	this->targetDistance = 0.0;
	this->targetAngle = 0.0;
	this->sumDistance = 0.0;
	this->sumAngle = 0.0;
	this->start = nullptr;
	this->end = nullptr;
}

void MotionPlanner::pidReset() {
	this->pidInit(this->posTolerance, this->angleTolerance, this->maxSpeed, this->maxAngleSpeed);
}

void MotionPlanner::pidSetTarget(double distance, double angle)
{
	targetDistance = distance;
	targetAngle = angle;
}

void MotionPlanner::pidSetTarget(Point target, Position position)
{
	float distToTarget = Math::getDistance(Point(position.x, position.y), target);
	float rotationToTarget = getRotationToTarget(Point(position.x, position.y), target, position.o);

	this->pidSetTarget(distToTarget, rotationToTarget);
}

void MotionPlanner::publishVelocity(double speed, double angle) {
	geometry_msgs::Twist msg;

	msg.linear.x = speed;
	msg.angular.z = angle;

	motionPub->publish(msg);
}

void PidController::pidUpdate(Position *current)
{
	double newAngle = 0.0;
	double newSpeed = 0.0;

	if (waypointReached(current))
	{
		ROS_INFO("GOAL ACHIEVED");
		publishVelocity(0.0, 0.0);
		return;
	}

	if (start == nullptr && last == nullptr)
	{
		*start = *current;
		*last = *current;
	}

	//Calculation of action intervention.
	if (fabs(targetDistance) > posTolerance)
	{
		newSpeed = pidCalculate(current, start->getDistance(current) * copysign(1.0, targetDistance), start->getDistance(last) * copysign(1.0, targetDistance), targetDistance, F_KP, F_KD, F_KI, &sumDistance);
	}

	if (current->o - last->o < -M_PI)
	{
		current->o += 2 * M_PI;
	}
	else if (current->o - last->o > M_PI)
	{
		current->o -= 2 * M_PI;
	}

	newAngle = pidCalculate(current, current->o - start->o, last->o - start->o, targetAngle, R_KP, R_KD, R_KI, &sumAngle);

	*last = *current;

	// publish velocity message
	publishVelocity(fmin(maxSpeed, newSpeed), fmin(maxAngleSpeed, newAngle));
}

bool PidController::waypointReached(Position *current) {
	double distance;
	distance = start->getDistance(current) * copysign(1.0, targetDistance);

	if (fabs(distance - targetDistance) > posTolerance)
	{
		return false;
	}

	if (fabs(targetAngle - (current->o - start->o)) > angleTolerance &
		fabs(targetAngle - (current->o - start->o) + 2 * M_PI) > angleTolerance &
		fabs(targetAngle - (current->o - start->o) - 2 * M_PI) > angleTolerance)
	{
		return false;
	}

	return true;
}

double MotionPlanner::pidCalculate(Position *current, double currentValue, double lastValue, double referenceValue, double kP, double kD, double kS, double *sum) {
	double speed = 0;
	double error = referenceValue - currentValue;
	double previousError = referenceValue - lastValue;
	double dt = current->t.toSec() - last->t.toSec();
	double derivative = (error - previousError) / dt;
	*sum = *sum + error * dt;
	//speed = kP*error + kD*derivative + kS*(*sum);
	speed = kP * error + kS * (*sum);

	return speed;
}

bool MotionPlanner::driveCurrentPath(Point currentPosition, double orientation) {
	geometry_msgs::Twist motion;

	float distToTarget = Math::getDistance(currentPosition, currentTarget);
	float rotationToTarget = getRotationToTarget(currentPosition, currentTarget, orientation);
	
	float rotationSign = rotationToTarget < 0.f ? -1.f : 1.f;
	rotationToTarget = std::fabs(rotationToTarget);
	
	if(rotationToTarget < allowedRotationDifference) {
		rotationToTarget = 0;
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
	double direction = std::atan2(targetPosition.y - currentPosition.y, targetPosition.x - currentPosition.x);

	return static_cast<float>(Math::getAngleDifferenceInRad(orientation, direction));
}

