#include <utility>
#include <include/agent/MotionPlanner.h>

#include "ros/ros.h"

#include "agent/Agent.h"


MotionPlanner::MotionPlanner(Agent* a, auto_smart_factory::RobotConfiguration robot_config, ros::Publisher* motion_pub) :
	pathObject(Path())
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

	steerPid = new PidController(0.0, 1.6, 0.0, 5.0);
}

MotionPlanner::~MotionPlanner() {
	delete steerPid;
};

void MotionPlanner::update(geometry_msgs::Point position, double orientation) {
	pos.update(position.x, position.y, orientation, ros::Time::now());
	positionInitialized = true;

	/* If motion planner is stopped or finished, set velocity to zero */
	if (mode == Mode::FINISHED || mode == Mode::STOP) {
		publishVelocity(0.0, 0.0);
		return;
	}

	/* Align towards a point or direction (orientation) */
	if (mode == Mode::ALIGN) {
		if (alignDirection == -1.0) {
			turnTowards(alignTarget);
		} else {
			turnTowards(alignDirection);
		}
		return;
	}

	/* Drive straight forward/backward */
	if (mode == Mode::FORWARD || mode == Mode::BACKWARD) {
		driveStraight();
		return;
	}

	/* Check if path is valid, as it is required for the following procedures */
	if (!pathObject.isValid()) {
		publishVelocity(0.0, 0.0);
		mode = Mode::FINISHED;
		return;
	}

	/* Check if the current target waypoint is reached and advance to next one or stop if it is the last one */
	if (isWaypointReached()) {
		if (!isCurrentPointLastPoint())	{
			advanceToNextPathPoint();
		} else {
			mode = Mode::FINISHED;
			publishVelocity(0.0, 0.0);
			publishEmptyVisualisationPath();
			return;
		}
	}

	/* Turn towards target orientation on spot when curve angle is above turnThreshold */
	double rotationDifference = std::abs(getRotationToTarget(pos, currentTarget));
	if (mode == Mode::TURN || rotationDifference >= turnThreshold || (rotationDifference >= turnThresholdFirstPoint && currentTargetIndex < 2)) {
		mode = Mode::TURN;
		turnTowards(currentTarget);
		return;
	}

	if (previousTargetIndex >= 0 && pathObject.getDepartureTimes().at(previousTargetIndex) > ros::Time::now().toSec()) {
		/* While waiting already turn into target direction to not waste time and if finished set linear and angular velocity to zero */
		turnTowards(currentTarget);

		/* If mode is driving go into WAIT mode until departure time is reached, turnTowards() will either rotate or if not necessary set velocity to zero */
		if (mode == Mode::PID || mode == Mode::READY) {
			mode = Mode::WAIT;
		}
		return;
	}

	/* Follow the path using the pid controller */
	followPath();
}

void MotionPlanner::followPath() {
	mode = Mode::PID;
	
	double cte = Math::getDistanceToLine(previousTarget, currentTarget, Point(pos.x, pos.y)) * Math::getDirectionToLineSegment(previousTarget, currentTarget, Point(pos.x, pos.y));
	double angularVelocity = steerPid->calculate(cte, ros::Time::now().toSec());
	angularVelocity = Math::clamp(angularVelocity, (double) -maxTurningSpeed, (double) maxTurningSpeed);

	double linearVelocity = maxDrivingSpeed - std::min((std::exp(cte*cte) - 1.0), (double) maxDrivingSpeed - minDrivingSpeed);
	linearVelocity = Math::clamp(linearVelocity, minDrivingSpeed, maxDrivingSpeed);

	// Limit speed if approaching final point
	double distToTarget = Math::getDistance(Point(pos.x, pos.y), currentTarget);
	if (isCurrentPointLastPoint() && distToTarget <= distToSlowDown) {
		linearVelocity = Math::clamp(distToTarget * 1.5f, minPrecisionDrivingSpeed, maxDrivingSpeed);
	}

	publishVelocity(linearVelocity, angularVelocity);
}

void MotionPlanner::turnTowards(Point target) {
	double rotation = getRotationToTarget(pos, target);
	if(std::abs(rotation) <= 0.02f) {
		/* If in align mode, the task is finished after rotation. If not, the robot should continue driving afterwards */
		publishVelocity(0, 0);
		if(mode == Mode::ALIGN) {
			mode = Mode::FINISHED;
		} else {
			mode = Mode::READY;
		}
		return;
	}
	publishVelocity(0, Math::clamp(std::abs(rotation) * 2.f, 0.2f, maxTurningSpeed) * (rotation < 0.f ? -1.f : 1.f));
}

void MotionPlanner::turnTowards(double direction) {
	double rotation = Math::getAngleDifferenceInRad(pos.o, direction);
	if(std::abs(rotation) <= 0.02f) {
		/* If in align mode, the task is finished after rotation. If not, the robot should continue driving afterwards */
		publishVelocity(0, 0);
		if(mode == Mode::ALIGN) {
			mode = Mode::FINISHED;
		} else {
			mode = Mode::READY;
		}
		return;
	}
	publishVelocity(0, Math::clamp(std::abs(rotation) * 2.f, 0.2f, maxTurningSpeed) * (rotation < 0.f ? -1.f : 1.f));
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

void MotionPlanner::driveForward(double distance) {
	mode = Mode::FORWARD;
	driveDistance = distance;
	driveStartPosition = pos;
}

void MotionPlanner::driveBackward(double distance) {
	mode = Mode::BACKWARD;
	driveDistance = distance;
	driveStartPosition = pos;
}

void MotionPlanner::driveStraight() {
	double currentDistance = Math::getDistance(Point(driveStartPosition.x, driveStartPosition.y), Point(pos.x, pos.y));
	if(currentDistance >= driveDistance - 0.015f) {
		mode = Mode::FINISHED;
		publishVelocity(0.0, 0.0);
		return;
	}

	if(mode == Mode::FORWARD) {
		publishVelocity(Math::clamp(currentDistance * 1.5f, minPrecisionDrivingSpeed, maxDrivingSpeed), 0.0);
	} else {
		publishVelocity(-Math::clamp(currentDistance * 1.5f, minPrecisionDrivingSpeed, maxDrivingSpeed), 0.0);
	}
}

void MotionPlanner::newPath(Path path) {
	pathObject = path;
	
	if(path.isValid()) {
		currentTarget = pathObject.getNodes().front();
		currentTargetIndex = 0;
		mode = Mode::READY;
		agent->getVisualisationPublisher()->publish(pathObject.getVisualizationMsgLines(agent->getAgentColor()));
	} else {
		ROS_ERROR("[MotionPlanner - %s]: Got invalid path", agentID.c_str());
		mode = Mode::FINISHED;
	}
}

void MotionPlanner::advanceToNextPathPoint() {
	previousTarget = pathObject.getNodes().at(static_cast<unsigned long>(currentTargetIndex));
	previousTargetIndex = currentTargetIndex;
	currentTargetIndex++;
	currentTarget = pathObject.getNodes().at(static_cast<unsigned long>(currentTargetIndex));
}

bool MotionPlanner::isCurrentPointLastPoint() {
	return (currentTargetIndex == pathObject.getNodes().size() - 1) && pathObject.isValid();
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
	return mode == Mode::FINISHED;
}

bool MotionPlanner::hasPath() {
	return pathObject.isValid();
}

bool MotionPlanner::isDrivingBackwards() {
	return false;
}

bool MotionPlanner::isWaypointReached() {
	if (!isCurrentPointLastPoint())	{
		return (Math::getDistance(Point(pos.x, pos.y), currentTarget) <= distToReachPoint);
	} else {
		return (Math::getDistance(Point(pos.x, pos.y), currentTarget) <= distToReachFinalPoint);
	}
}

void MotionPlanner::publishVelocity(double speed, double angle) {
	geometry_msgs::Twist msg;

	msg.linear.x = speed;
	msg.angular.z = angle;

	motionPub->publish(msg);
}

double MotionPlanner::getRotationToTarget(Position currentPosition, Point targetPosition) {
	double direction = std::atan2(targetPosition.y - currentPosition.y, targetPosition.x - currentPosition.x);

	return Math::getAngleDifferenceInRad(currentPosition.o, direction);
}

OrientedPoint MotionPlanner::getPositionAsOrientedPoint() {
	return OrientedPoint(pos.x, pos.y, pos.o);
}

bool MotionPlanner::isPositionInitialized()  {
	return positionInitialized;
}

void MotionPlanner::publishEmptyVisualisationPath() {
	visualization_msgs::Marker msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.ns = "PathLines";
	msg.action = visualization_msgs::Marker::ADD;
	msg.id = 1;
	agent->getVisualisationPublisher()->publish(msg);
}
