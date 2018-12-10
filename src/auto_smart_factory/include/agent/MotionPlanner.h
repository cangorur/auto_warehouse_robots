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

	/* Enables / disables this motion planner.
	 * @param enable: whether to enable or disable this motion planner*/
	void enable(bool enable);

	/* Returns if this motion planner is enabled right now.
	 * @return True if this motion planner is enabled*/
	bool isEnabled();

	/* Once the tests are enabled, the regular operation stops and robot can only be controlled manually
	 * Returns if this motion planner is enabled for the tests.
	 * @return True if this motion planner is enabled for testing*/
	bool isTestsEnabled();

	/* Sets the motion planner to be allowed to move.*/
	void start();

	/* Sets the motion planner to be not allowed to move and tells the motion acuator to stand still.*/
	void stop();

	/* Returns whether this motion planner has succesfully driven the current path.
	 * @return True whether the current path has been driven*/
	bool isDone();

	/* Returns if this motion planner currently holds a valid path.
	 * @return True if this motion planner currently holds a valid path*/
	bool hasPath();

	/* Returns if this motion planner is driving backwards right now.
	 * @return True if this motion planner is driving backwards*/
	bool isDrivingBackwards();

	void emgRetreat(float stop_interval, float retreat_interval);

private:
	/* Hands over the currently sensed position and orientation of the robot to execute the driving
	 * of the current path. Manages the current path and decides wheter to turn or not and publishes
	 * the velocities to the motion actuator topic.
	 * @param position: current position of the robot
	 * @param orientation: current oriientation of the robot
	 * @return True if the current plan has been driven successfully*/
	bool driveCurrentPath(Point currentPosition, double orientation);
	
	/* Returns the direction between the given 2 points considering p1 the center point and p2 the
	 * target point.
	 * @returns direction e.g.
	 *       0.5
	 *        ||
	 *  1/-1--  --0
	 *        ||
	 *      -0.5*/
	double getDirection(double p1_x, double p1_y, double p2_x, double p2_y);

	/* Returns the distance between 2 points.
	 * @return distance in meters*/
	double getDistance(double p1_x, double p1_y, double p2_x, double p2_y);

	/* Calculates the orientation difference between the given orientation and the given direction.
	 * If difference is positive the robot should turn counter-clockwise
	 * to head to the direction as far as possible, else negative -> clockwise.
	 * @param orientation: real orientation e.g.
	 * @param direction: desired direction e.g.
	 * @return orientation difference given as direction e.g.
	 *       0.5
	 *        ||
	 *  1/-1--  --0
	 *        ||
	 *      -0.5*/
	double getOrientationDiff(double orientation, double direction);

	/* Returns the angle of the given orientation.
	 * @param orientation: orientation e.g
	 *       0.5
	 *        ||
	 *  1/-1--  --0
	 *        ||
	 *      -0.5
	  @returns an orientation angle e.g.
	 *      90
	 *      ||
	 * 180--  --0
 	 *      ||
	 *     270*/
	double getAngle(double orientation);
	
	float getAngleFromOrientation(double orientation);
	
	// Helper functions
	float clamp(float value, float min, float max) const;
	float lerp(float start, float end, float alpha) const;

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

	/// the index of the current waypoint
	//int currentTargetIndex = -1;

	/// the point the robot should look to, when the goal position is reached
	geometry_msgs::Point endDirectionPoint;

	/// Flag whether the motion planner is enabled or not
	bool enabled = false;

	/// enable ros param string
	std::string test_enable_paramStr;

	/// Flag whether the tests are enabled: allows to control the robot manually
	bool tests_enabled = false;

	/// Flag whether the motion planner has succesfully driven the current path
	bool done = true;

	/// Flag whether the motion planner should stop moving
	bool standStill = true;

	/// Flag whether the motion planner is turn on spot
	bool turning = false;

	/// Flag whether to drive the current backwards or not
	bool driveBackwards = false;

	double turningMinSpeed = 0.02;
	double turningMaxSpeed = 1;
	double drivingMinSpeed = 0.2;
	double drivingMaxSpeed = 1;

	/// current turning speed of the robot - left > 0 > right
	double turnDirection = 0;
	/// Flag whether turning just started
	bool turningStart = true;
	/// Flag whether the robot turns left right now
	bool turnsLeft = true;
	/// Number of turns the robot did while current turning process
	int numberTurns = 0;

	/* Accuracy that determines the allowed distance to stop when in vicinity of the goal point while
	 * driving forward.*/
	double driveForwardDistanceAccuracy = 0.15;

	/* Accuracy that determines the allowed distance to stop when in vicinity of the goal point while
	 * driving backwards.*/
	double driveBackwardsDistanceAccuracy = 0.3;

	/* Accuracy that determines the allowed orientaion difference of the current robot
	 * orientation while driving & the direction of the goal position to not steer.*/
	double steeringAccuracy = 0.01;

	/* Accuracy that determines the allowed orientation difference
	 * to stop if the difference is small enough while turning on spot.*/
	double turnOnSpotAccuracy = 0.04;   // from 0.02

	/// pi
	const double PI = 3.141592653589793238463;

protected:
	/// The agent to which this motion planner belongs
	Agent* agent;

	/// agent ID variable set by Agent.cpp
	std::string agentID;

};

#endif /* AUTO_SMART_FACTORY_SRC_MOTIONPLANNER_H_ */
