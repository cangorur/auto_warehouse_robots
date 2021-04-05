#ifndef AUTO_SMART_FACTORY_SRC_MOTIONPLANNER_H_
#define AUTO_SMART_FACTORY_SRC_MOTIONPLANNER_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "auto_smart_factory/RobotConfiguration.h"
#include "agent/path_planning/Path.h"
#include "agent/path_planning/Point.h"
#include "agent/Position.h"
#include "agent/PidController.h"

#include "Math.h"

class Agent;

/* 
 * The Motion Planner controls the velocity (linear and angular) of the robot.
 * It takes a path as a list of points and follows the path given the precision contraints set in the header file.
 * The Motion Planner takes the poseCallback in the Agent.h to call it's update function whenever the robots position changes.
 */
class MotionPlanner {

public:
	/*
	 * Contains the current mode of the motion planner
	 * READY: The motion planner has a valid path and is ready to drive. This state will automatically turn into PID, TURN or FINISHED in the next iteration
	 * PID: The pid controller is currently in charge of the robots motion
	 * TURN: The robot performs an on-spot-turn while following a path and will become READY as soon as the turn is finished
	 * FINISHED: The robot reached the last waypoint of the path
	 * STOP: The robot was stopped manually - e.g., by calling stop()
	 * ALIGN: The robot should align towards a target or into a direction manually - e.g., by calling alignTowards(...)
	 * WAIT: The robots waits for the departure time of a path point to be reached
	 * FORWARD: The robot drives forward for a manually given distance - e.g. by calling driveForward(distance)
	 * BACKWARD: The robot drives backward for a manually given distance e.g. by calling driveBackward(distance)
	 */ 
	enum class Mode {READY, PID, TURN, FINISHED, STOP, ALIGN, WAIT, FORWARD, BACKWARD};

	/* Constructor that hands over some the robot configuration as well as
	 * the motion acutator publisher.
	 * @param robot_config: information about the role of the agent this motion planner belongs to
	 * @param motion_pub: publisher for the motion actuator topic*/
	MotionPlanner(Agent* agent, auto_smart_factory::RobotConfiguration robot_config, ros::Publisher* motion_pub);

	/* 
	 * Updates the current position and orientation of the robot given the input (usually called by the poseCallback)
	 * Performs the driving operation based on the current Mode and adjusts the mode accordinly
	 * @param position: current position of the agent
	 * @param orientation: current orientation of the agent*/
	void update(geometry_msgs::Point position, double orientation);

	/* 
	 * Align the robots towards a target or direction 
	 * Variant 1: @param Point target to align towards that target
	 * Variant 2: @param double direction to align towards a direction 
	 */
	void alignTowards(Point target);
	void alignTowards(double direction);

	/* 
	 * Drive straight forward or backward for given distance 
	 * @param double distance the distance to drive
	 */
	void driveForward(double distance);
	void driveBackward(double distance);

	/* 
	 * Give the Motion Planner a new path to drive. 
	 * The Motion Planner will move into READY state, if the path is valid
	 */
	void newPath(Path path);

	/*
	 * Returns the current mode of the Motion Planner
	 * @return Mode mode
	 */
	Mode getMode();

	/*
	 * Start: Set the robot state to READY
	 * Stop: Set the robot state to STOP
	 */
	void start();
	void stop();

	/*
	 * Returns true if the Motion Planner is in FINISHED state
	 */ 
	bool isDone();

	/*
	 * Returns true if the Motion Planner is in STOP state
	 */
	bool isStopped();

	/*
	 * Returns true if the Motion Planner has a current path
	 * This will also be true, if the path is already driven
	 */
	bool hasPath();

	/*
	 * Returns true if the current linear velocity is below zero
	 */
	bool isDrivingBackwards();

	/*
	 * Return the current Position of the robot as an OrientedPoint
	 */
	OrientedPoint getPositionAsOrientedPoint();

	/*
	 * Returns true, if the position of the robot has been updated at least once (i.e. the poseCallback called update() at least once)
	 */
	bool isPositionInitialized();

	/*
	 * virtually destructs the MotionPlanner
	 */
	virtual ~MotionPlanner();

  private:
	/* 
	 * Follow a path using the pid controller
	 */
	void followPath();

	/* 
	 * Variant 1: Turn the robot on spot facing towards the given target when finished
	 * Variant 2: Turn the robot on spot facing the given direction when finished 
	 */
	void turnTowards(Point target);
	void turnTowards(double direction);

	/* 
	 * Drive the robot straight forward or backward based on mode 
	 * for given driveDistance (set with driveForward() or driveBackward()) 
	 */
	void driveStraight();

	/* 
	 * Returns true, if the robot is within distToReachPoint / distToReachFinalPoint
	 */
	bool isWaypointReached();

	/*
	 * Returns true, if the current point is the last point in the path
	 */
	bool isCurrentPointLastPoint();

	/*
	 * Set the currentTarget to the next point in the path
	 */
	void advanceToNextPathPoint();
	
	/* 
	 * Calculate the rotation angle to the target (in rad) given
	 * @param Position currentPosition the robots current position
	 * @param Point targetPosition the target position
	 */
	double getRotationToTarget(Position currentPosition, Point targetPosition);

	/*
	 * Calculate the distance to the final waypoint
	 * Summing the distances between all waypoints
	 */
	double getDistanceToTarget();

	/*
	 * Pusblish velocity to the motionPub topic and update class attributes for liner and angular velocity
	 */
	void publishVelocity(double speed, double angle);
	
	/*
	 * Removes last published path in RVIZ when path is finished
	 */
	void publishEmptyVisualisationPath();

	// information about the current role of the agent
	auto_smart_factory::RobotConfiguration robotConfig;

	// Publisher for the motion actuator topic
	ros::Publisher* motionPub;

	// Publisher for the visualization messages (rviz)
	ros::Publisher pathPub;

	// the current mode
	Mode mode = Mode::STOP;

	// most recent published linear and angular velocity
	double currentLinearVelocity = 0.0;
	double currentAngularVelocity = 0.0;
	
	// the current path of the robot
	Path pathObject;
	
	// the current target for the robot with its index
	Point currentTarget;
	int currentTargetIndex = -1;

	// the previous target for the robot with its index
	Point previousTarget;
	int previousTargetIndex = -1;

	// members for turnTowards functionality
	Point alignTarget;
	double alignDirection = 0;
	bool useDirectionForAlignment = false;

	// members for driveForward and driveBackward functionality
	Position driveStartPosition;
	double driveDistance = 0.0;

	// threshold when robot should stop and turn on spot instead of driving pid controlled
	double turnThreshold = Math::toRad(55.f);
	double turnThresholdFirstPoint = Math::toRad(15.f);
	
	// turning and driving speed limitations from robots config file
	float maxTurningSpeed;
	float minDrivingSpeed;
	float maxDrivingSpeed;

	// precision configuration to reach points accurately
	float distToReachPoint = 0.3f;
	float distToReachFinalPoint = 0.02f;
	float distToSlowDown = 0.7f;
	float minPrecisionDrivingSpeed = 0.1f;

	// will be true when position is updated the first time
	bool positionInitialized = false;

protected:
	Agent* agent;
	std::string agentID;

	// the pid controller instance controlling the angular velocity
	PidController* steerPid;

	// the current position of the robot
	Position pos;
};

#endif /* AUTO_SMART_FACTORY_SRC_MOTIONPLANNER_H_ */
