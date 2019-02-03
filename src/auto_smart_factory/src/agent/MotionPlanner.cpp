#include <utility>
#include "ros/ros.h"

#include "agent/Agent.h"
#include <visualization_msgs/Marker.h>

#include "agent/MotionPlanner.h"
#include <cmath>


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

	steerPid = new PidController(0.0, 1.8, 0.0, 4.0);
}

MotionPlanner::~MotionPlanner() {
	delete steerPid;
};

void MotionPlanner::update(geometry_msgs::Point position, double orientation) {
	pos.update(position.x, position.y, orientation, ros::Time::now());
	positionInitialized = true;

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

	/* Check if the current target waypoint is reached and advance to next one or stop if it is the last one */
	if (isWaypointReached()) {
		if (!isCurrentPointLastPoint())	{
			advanceToNextPathPoint();
		} else {
			mode = Mode::FINISHED;
			publishVelocity(0.0, 0.0);
			
			agent->getVisualisationPublisher()->publish(pathObject.getVisualizationMsgPoints());
			agent->getVisualisationPublisher()->publish(pathObject.getVisualizationMsgLines());
			return;
		}
	}

	/* Turn towards target orientation on spot when curve angle is above turnThreshold */
	if (mode == Mode::TURN || std::abs(getRotationToTarget(pos, currentTarget)) >= turnThreshold) {
		mode = Mode::TURN;
		turnTowards(currentTarget);
		return;
	}

	if (previousTargetIndex >= 0 && pathObject.getDepartureTimes().at(previousTargetIndex) > ros::Time::now().toSec()) {
		/* While waiting already turn into target direction to not waste time */
		turnTowards(currentTarget);

		/* If mode is driving, stop and go into WAIT mode until departure time is reached */
		if (mode == Mode::PID || mode == Mode::READY) {
			publishVelocity(0.0, 0.0);
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
	angularVelocity = std::min(std::max(angularVelocity, (double) -maxTurningSpeed), (double) maxTurningSpeed);

	double linearVelocity = maxDrivingSpeed - std::min((std::exp(cte*cte)-1), (double) maxDrivingSpeed-minDrivingSpeed);

	if (isCurrentPointLastPoint() && Math::getDistance(Point(pos.x, pos.y), currentTarget) < 0.4f) {
		linearVelocity = std::max(0.1, (double) Math::getDistance(Point(pos.x, pos.y), currentTarget));
	}

	publishVelocity(linearVelocity, angularVelocity);
}

void MotionPlanner::turnTowards(Point target) {
	double rotation = getRotationToTarget(pos, target);
	if(std::abs(rotation) <= 0.1f) {
		/* If in align mode, the task is finished after rotation. If not, the robot should continue driving afterwards */
		if(mode == Mode::ALIGN) {
			mode = Mode::FINISHED;
			publishVelocity(0, 0);
		} else {
			mode = Mode::READY;
		}
		return;
	}
	publishVelocity(0, Math::clamp(std::abs(rotation), 0, maxTurningSpeed) * (rotation < 0.f ? -1.f : 1.f));
}

void MotionPlanner::turnTowards(double direction) {
	double rotation = static_cast<double>(Math::getAngleDifferenceInRad(pos.o, direction));
	if(std::abs(rotation) <= 0.1f) {
		/* If in align mode, the task is finished after rotation. If not, the robot should continue driving afterwards */
		if(mode == Mode::ALIGN) {
			mode = Mode::FINISHED;
			publishVelocity(0, 0);
		} else {
			mode = Mode::READY;
		}
		return;
	}
	publishVelocity(0, Math::clamp(std::abs(rotation), 0.3, maxTurningSpeed) * (rotation < 0.f ? -1.f : 1.f));
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
	if(Math::getDistance(Point(driveStartPosition.x, driveStartPosition.y), Point(pos.x, pos.y)) >= driveDistance) {
		mode = Mode::FINISHED;
		publishVelocity(0.0, 0.0);
		return;
	}

	if(mode == Mode::FORWARD) {
		publishVelocity(0.5, 0.0);
	} else {
		publishVelocity(-0.5, 0.0);
	}
}

void MotionPlanner::newPath(Path path) {
	pathObject = path;
	
	if(path.isValid()) {
		currentTarget = pathObject.getNodes().front();
		currentTargetIndex = 0;
		mode = Mode::READY;
		agent->getVisualisationPublisher()->publish(pathObject.getVisualizationMsgPoints());
		agent->getVisualisationPublisher()->publish(pathObject.getVisualizationMsgLines());
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
	return (mode == Mode::FINISHED);
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

	return static_cast<double>(Math::getAngleDifferenceInRad(currentPosition.o, direction));
}

OrientedPoint MotionPlanner::getPositionAsOrientedPoint() {
	return OrientedPoint(pos.x, pos.y, pos.o);
}

bool MotionPlanner::isPositionInitialized()  {
	return positionInitialized;
}

visualization_msgs::Marker MotionPlanner::getVisualizationMsgPoints() {
	return pathObject.getVisualizationMsgPoints();
}

visualization_msgs::Marker MotionPlanner::getVisualizationMsgLines() {
	return pathObject.getVisualizationMsgLines();
}
