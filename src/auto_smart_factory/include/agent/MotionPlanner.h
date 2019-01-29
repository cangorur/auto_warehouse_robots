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
	enum class Mode {READY, PID, TURN, FINISHED, STOP, RECOVERY, ALIGN, WAIT};

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

	/* Align the robots towards a target / direction */
	void alignTowards(Point target);
	void alignTowards(double direction);

	/* Sets the current path to be driven by this motion planner and resets all necessary variables.
	 * @param start_position: the start position of the agent to drive the given plan
	 * @param new_path: the new path to drive
	 * @param end_direction_point: the point the agent should look to when the path has been
	 * 			       successfully driven
	 * @param drive_backwares: whether the given plan shall be driven backwards or not*/
	void newPath(geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> new_path,
	             geometry_msgs::Point end_direction_point, bool drive_backwards = false);

	void newPath(Path path);

	Mode getMode(void);

	void resume();
	void start();
	void stop();
	bool isDone();
	bool hasPath();
	bool isDrivingBackwards();

	visualization_msgs::Marker getVisualizationMsgPoints();
	visualization_msgs::Marker getVisualizationMsgLines();

private:
	/* Follow a path using the pid */
	void followPath(void);

	/* Turn the robot on spot facing orientation when finished */
	void turnTowards(Point target);
	void turnTowards(double direction);

	/* Functions to handle Waypoints */
	bool isWaypointReached(void);
	bool isCurrentPointLastPoint();

	void advanceToNextPathPoint();
	
	/* Calculate rotation to target */
	double getRotationToTarget(Position currentPosition, Point targetPosition);

	/* Helper function to publish the velocity on the robots motion topic */
	void publishVelocity(double speed, double angle);

	/// information about the current role of the agent
	auto_smart_factory::RobotConfiguration robotConfig;

	/// Publisher for the motion actuator topic
	ros::Publisher* motionPub;

	/// Publisher for the visualization messages (rviz)
	ros::Publisher pathPub;

	/// the current mode
	Mode mode = Mode::STOP;
	
	/// the current path of the robot
	Path pathObject;
	
	/// the current target for the robot with its index
	Point currentTarget;
	int currentTargetIndex = -1;

	Point previousTarget;
	int previousTargetIndex = -1;

	/// Members for turnTowards functionality
	Point alignTarget;
	double alignDirection = -1;

	/// Threshold when robot should stop and turn on spot instead of pid controlled
	double turnThreshold = Math::toRad(65);
	
	/// Turning and Driving Speed Limitations from robots config file
	float maxTurningSpeed;
	float minDrivingSpeed;
	float maxDrivingSpeed;

	/// Precision configuration to reach points
	float distToReachPoint = 0.3f;
	float distToReachFinalPoint = 0.1f;
	float distToSlowDown = 0.9f;

protected:
	Agent* agent;
	std::string agentID;

	PidController* steerPid;

	Position pos;

	double currentSpeed;
	double currentRotation;

};

#endif /* AUTO_SMART_FACTORY_SRC_MOTIONPLANNER_H_ */
