#ifndef AUTO_SMART_FACTORY_SRC_MOTIONPLANNER_H_
#define AUTO_SMART_FACTORY_SRC_MOTIONPLANNER_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include <auto_smart_factory/RobotConfiguration.h>
#include "agent/path_planning/Path.h"
#include "agent/path_planning/Point.h"
#include "agent/Position.h"
#include "agent/PidController.h"

#include "Math.h"

class Agent;

/* The motion planner component manages all movement related stuff ongoing at the agents side.
 * It coverts information about the current position as well as orientation and  a list of points
 * called path to a certain velocity and publishes it to the motion actuator topic to move the robot.
 * This implementation of motion planning only considers a differential drive and
 * is only able to drive straight forward or backwards or turn on spot - so no curve driving except of
 * some path correction while driving straight forward or backward.
 * The motion planner needs to be fed continously with current pose data to work as expected.*/
class MotionPlanner {

public:
	enum class Mode {READY, PID, TURN, FINISHED, STOP, RECOVERY, ALIGN};

	/* Constructor that hands over some the robot configuration as well as
	 * the motion acutator publisher.
	 * @param robot_config: information about the role of the agent this motion planner belongs to
	 * @param motion_pub: publisher for the motion actuator topic*/
	MotionPlanner(Agent* agent, auto_smart_factory::RobotConfiguration robot_config, ros::Publisher* motion_pub);

	virtual ~MotionPlanner();

	/* Hands over the current position as well as the current orientation of the robot on the current
	 * map and determines whether to drive the current path or not.
	 * @param position: current position of the agent
	 * @param orientation: current orientation of the agent*/
	void update(geometry_msgs::Point position, double orientation);

	/* Turn the robot on spot facing orientation when finished */
	void turnTowards(Point target);
	void turnTowards(double direction);

	/* Align the robots towards a target / direction */
	void alignTowards(Point target);
	void alignTowards(double direction);

	void newPath(Path* path);
	void newPath(Path path);

	Mode getMode();

	void start();
	void stop();
	bool isDone();
	bool hasPath();
	
	bool isDrivingBackwards();

	visualization_msgs::Marker getVisualizationMsgPoints();
	visualization_msgs::Marker getVisualizationMsgLines();

private:
	/* Hands over the currently sensed position and orientation of the robot to execute the driving
	 * of the current path. Manages the current path and decides wheter to turn or not and publishes
	 * the velocities to the motion actuator topic.
	 * @param position: current position of the robot
	 * @param orientation: current oriientation of the robot
	 * @return True if the current plan has been driven successfully*/
	bool driveCurrentPath(Position currentPosition);
	
	float getRotationFromOrientation(double orientation);
	float getRotationFromOrientationDifference(double orientation);
	
	bool isCurrentPointLastPoint();
	void advanceToNextPathPoint();
	Point getPathPointAtIndex(int index);
	
	double getRotationToTarget(Position currentPosition, Point targetPosition);

	void publishVelocity(double speed, double angle);

	bool waypointReached(Position* current);

	/// information about the current role of the agent
	auto_smart_factory::RobotConfiguration robotConfig;

	/// Publisher for the motion actuator topic
	ros::Publisher* motionPub;
	ros::Publisher pathPub;

	/// the current path to drive
	std::vector<geometry_msgs::Point> path;

	/// the current mode
	Mode mode = Mode::STOP;
	
	/////////////////////////////////////
	Path pathObject;
	
	Point currentTarget;
	int currentTargetIndex = -1;

	Point previousTarget;

	Point alignTarget;
	double alignDirection;

	double turnThreshold = Math::toRad(65);

	float minTurningSpeed = 0.08;
	
	float maxTurningSpeed; // = 1;
	float minDrivingSpeed; // = 0.2;
	float maxDrivingSpeed; // = 1;

	float distToReachPoint = 0.3f;
	float distToReachFinalPoint = 0.1f;
	float distToSlowDown = 0.9f;

	//float maxRotationDifference = 45;
	float maxRotationDifference = 0.38f;
	
	/* Accuracy that determines the allowed orientaion difference of the current robot
	 * orientation while driving & the direction of the goal position to not steer.*/
	float allowedRotationDifference = 0.001f;


protected:
	Agent* agent;
	std::string agentID;

	PidController* steerPid;

	Position pos;

	double currentSpeed;
	double currentRotation;

};

#endif /* AUTO_SMART_FACTORY_SRC_MOTIONPLANNER_H_ */
