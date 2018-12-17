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

#define F_KP 2.58  // P constant for PSD translation controller
#define F_KD 0.047 // D constant for PSD translation controller
#define F_KI 0.0   // S constant for PSD translation controller
#define R_KP 2.0   // P constant for PSD rotation controller
#define R_KD 0.1   // D constant for PSD rotation controller
#define R_KI 0.0   // S constant for PSD rotation controller

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

	/* Sets the current path to be driven by this motion planner and resets all necessary variables.
	 * @param start_position: the start position of the agent to drive the given plan
	 * @param new_path: the new path to drive
	 * @param end_direction_point: the point the agent should look to when the path has been
	 * 			       successfully driven
	 * @param drive_backwares: whether the given plan shall be driven backwards or not*/
	void newPath(geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> new_path,
	             geometry_msgs::Point end_direction_point, bool drive_backwards = false);

	void enable(bool enable);
	bool isEnabled();
	void start();
	void stop();
	bool isDone();
	bool hasPath();
	
	bool isDrivingBackwards();

private:
	/* Hands over the currently sensed position and orientation of the robot to execute the driving
	 * of the current path. Manages the current path and decides wheter to turn or not and publishes
	 * the velocities to the motion actuator topic.
	 * @param position: current position of the robot
	 * @param orientation: current oriientation of the robot
	 * @return True if the current plan has been driven successfully*/
	bool driveCurrentPath(Point currentPosition, double orientation);
	
	float getRotationFromOrientation(double orientation);
	float getRotationFromOrientationDifference(double orientation);
	
	bool isCurrentPointLastPoint();
	void advanceToNextPathPoint();
	
	float getRotationToTarget(Point currentPosition, Point targetPosition, double orientation);

	/* PID Controller methods */
	void pidInit(double posTolerance, double angleTolerance, double maxSpeed, double maxAngleSpeed);
	void pidReset(void);

	void pidSetTarget(double distance, double angle);
	void pidSetTarget(Point target, Position position);

	void publishVelocity(double speed, double angle);

	void pidUpdate(Position* pos);

	bool waypointReached(Position* current);

	double pidCalculate(Position* current, double currentValue, double lastValue, double referenceValue, double kP, double kD, double kS, double* sum);

		/// information about the current role of the agent
	auto_smart_factory::RobotConfiguration robotConfig;

	/// Publisher for the motion actuator topic
	ros::Publisher* motionPub;
	ros::Publisher pathPub;

	/// the current path to drive
	std::vector<geometry_msgs::Point> path;
	
	/////////////////////////////////////
	Path pathObject;
	
	Point currentTarget;
	int currentTargetIndex = -1;

	bool enabled = false;
	bool hasFinishedCurrentPath = true;
	bool standStill = true;

	float minTurningSpeed = 0.08;
	
	float maxTurningSpeed; // = 1;
	float minDrivingSpeed; // = 0.2;
	float maxDrivingSpeed; // = 1;

	float distToReachPoint = 0.2f;
	float distToReachFinalPoint = 0.2f;
	float distToSlowDown = 0.9f;

	//float maxRotationDifference = 45;
	float maxRotationDifference = 0.38f;
	
	/* Accuracy that determines the allowed orientaion difference of the current robot
	 * orientation while driving & the direction of the goal position to not steer.*/
	float allowedRotationDifference = 0.001f;

	/* PID Controller Attributes */
	Position *pidStart;
	Position *pidLast;
	double maxSpeed;
	double maxAngleSpeed;
	double posTolerance;
	double angleTolerance;
	double pidTargetDistance;
	double pidTargetAngle;
	double pidSumDistance;
	double pidSumAngle;


protected:
	Agent* agent;
	std::string agentID;
};

#endif /* AUTO_SMART_FACTORY_SRC_MOTIONPLANNER_H_ */
