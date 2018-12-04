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

class Agent;

/**
 * The motion planner component manages all movement related stuff ongoing at the agents side.
 * It coverts information about the current position as well as orientation and  a list of points
 * called path to a certain velocity and publishes it to the motion actuator topic to move the robot.
 * This implementation of motion planning only considers a differential drive and
 * is only able to drive straight forward or backwards or turn on spot - so no curve driving except of
 * some path correction while driving straight forward or backward.
 * The motion planner needs to be fed continously with current pose data to work as expected.
 */
class MotionPlanner {

public:
	/**
	 * Constructor that hands over some the robot configuration as well as
	 * the motion acutator publisher.
	 * @param robot_config: information about the role of the agent this motion planner belongs to
	 * @param motion_pub: publisher for the motion actuator topic
	 */
	MotionPlanner(Agent* agent, auto_smart_factory::RobotConfiguration robot_config, ros::Publisher* motion_pub);

	virtual ~MotionPlanner();

	/**
	 * Hands over the current position as well as the current orientation of the robot on the current
	 * map and determines whether to drive the current path or not.
	 * @param position: current position of the agent
	 * @param orientation: current orientation of the agent
	 */
	void update(geometry_msgs::Point position, double orientation);

	/**
	 * Sets the current path to be driven by this motion planner and resets all necessary variables.
	 * @param start_position: the start position of the agent to drive the given plan
	 * @param new_path: the new path to drive
	 * @param end_direction_point: the point the agent should look to when the path has been
	 * 			       successfully driven
	 * @param drive_backwares: whether the given plan shall be driven backwards or not
	 */
	void newPath(geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> new_path,
	             geometry_msgs::Point end_direction_point, bool drive_backwards = false);

	/**
	 * Smooths a path provided as input
	 * @param path: the path to smooth
	 */
	void smoothPath(std::vector<geometry_msgs::Point>& path, double weight_data = 0.5, double weight_smooth = 0.1,
	                double tolerance = 0.000001);

	/**
 	 * Returns if the given path is equal to the current path.
	 * @param new_path: the path to compare the current path with
	 * @return True if the paths are equal
	 */
	bool comparePaths(std::vector<geometry_msgs::Point> new_path);

	/**
	 * Enables / disables this motion planner.
	 * @param enable: whether to enable or disable this motion planner
	 */
	void enable(bool enable);

	/**
	 * Returns if this motion planner is enabled right now.
	 * @return True if this motion planner is enabled
	 */
	bool isEnabled();

	/**
	 * Once the tests are enabled, the regular operation stops and robot can only be controlled manually
	 * Returns if this motion planner is enabled for the tests.
	 * @return True if this motion planner is enabled for testing
	 */
	bool isTestsEnabled();

	/**
	 * Sets the motion planner to be allowed to move.
	 */
	void start();

	/**
	 * Sets the motion planner to be not allowed to move and tells the motion acuator to stand still.
	 */
	void stop();

	/**
	 * Returns whether this motion planner has succesfully driven the current path.
	 * @return True whether the current path has been driven
	 */
	bool isDone();

	/**
	 * Returns whether this motion planner is turning right now.
	 * @return True if this motion planner is turning on spot
	 */
	bool isTurning();

	/**
	 * Returns whether this motion planner is turning left right now.
	 * @return True if it is turning left
	 */
	bool isTurningLeft();

	/**
	 * Returns the velocity of the z-axis turning.
	 * @return the current turning velocity: left > 0 > right
	 */
	double getTurnDirection();

	/**
	 * Changes the direction the robot should turn to in terms of providing collisions while turning.
	 */
	void changeTurningDirection();

	/**
	 * Returns number of changes of direction turns done so far while this current turning process.
	 * @return number of turns
	 */
	int getNumberTurns();

	/**
	 * Returns if this motion planner currently holds a valid path.
	 * @return True if this motion planner currently holds a valid path
	 */
	bool hasPath();

	/**
	 * Returns the path that consists of the remaining points the agent has to drive to
	 * fullfill driging the current path.
	 * @return remaining path
	 */
	std::vector<geometry_msgs::Point> getRemainingPath();

	/**
	 * Returns if this motion planner is driving backwards right now.
	 * @return True if this motion planner is driving backwards
	 */
	bool isDrivingBackwards();

	/**
	 * Compares the current paths goal with the current paths goal.
	 * @param new_path: the path to compare the current path with
	 * @return True if the current path has a different goal than the give path
	 */
	bool hasDifferentGoal(std::vector<geometry_msgs::Point> new_path);

	/**
	 * Returns the last waypoint position the robot has been to.
	 * @return waypoint where the agent previously has been
	 */
	geometry_msgs::Point getLastPosition();

	/**
	 * Returns the next waypoint the agent is going to drive to.
	 * @return waypoint the agents drives to next
	 */
	geometry_msgs::Point getNextPosition();

	/**
	 * Returns the goal position of the current path.
	 * @return goal position of the current path
	 */
	geometry_msgs::Point getGoalPosition();

	/**
	 * Returns the end direction point of the current path.
	 * @return point the agent should look at when the the goal has been reached
	 */
	geometry_msgs::Point getEndDirectionPoint();

	/**
	 * Returns the drive forward distance accuracy.
	 * @return drive forward distance accuracy
	 */
	double getDriveForwardDistanceAccuracy();

	/**
	 * Returns the drive backward distance accuracy.
	 * @return drive backward distance accuracy
	 */
	double getDriveBackwardsDistanceAccuracy();


	void emgRetreat(float stop_interval, float retreat_interval);

// protected:
	/**
	 * Hands over the currently sensed position and orientation of the robot to execute the driving
	 * of the current path. Manages the current path and decides wheter to turn or not and publishes
	 * the velocities to the motion actuator topic.
	 * @param position: current position of the robot
	 * @param orientation: current oriientation of the robot
	 * @return True if the current plan has been driven successfully
	 */
	bool driveCurrentPath(geometry_msgs::Point position, double orientation);

	/**
	 * Calculates the velocities to drive from the given position to the the given goal point
	 * considering the given orientation of the robot to correct the current path to the target.
	 * Executes the next waypoint method when the target has been reached.
	 * @param p_x: x coordinate of the current robot position
	 * @param p_y: y coordinate of the current robot position
	 * @param t_x: x coordinate of the target location
	 * @param t_y: y coordinate of the target location
	 * @param orientation: current orientation of the robot
	 * @return calculated velocities
	 */
	geometry_msgs::Twist driveStraight(double p_x, double p_y, double t_x, double t_y,
	                                   double orientation);

	/**
	 * Increases the current target index and resets turning related variables.
	 */
	void nextWayPoint();

	/**
	 * Calculates the velocities to turn the robot on spot to look to the given goal point
	 * considering the given orientation of the robot.
	 * @param p_x: x coordinate of the current robot position
	 * @param p_y: y coordinate of the current robot position
	 * @param t_x: x coordinate of the goal point
	 * @param t_y: y coordinate of the goal point
	 * @param orientation: current orientation of the robot
	 * @return calculated turning velocities
	 */
	geometry_msgs::Twist turnOnSpot(double p_x, double p_y, double t_x, double t_y,
	                                double orientation);

	/**
	 * Scale the given speed to fit into the given constraints: min <= speed <= max.
	 * @param speed: the given speed
	 * @param max: the maximal speed allowed
	 * @param min: the minimal speed allowed
	 * @return the scaled speed
	 */
	double scaleSpeed(double speed, double max, double min);

	/**
	 * Returns the direction between the given 2 points considering p1 the center point and p2 the
	 * target point.
	 * @param p1_x: x coordinate of center point
	 * @param p1_y: y coordinate of center point
	 * @param p2_x: x coordinate of target point
	 * @param p2_y: y coordinate of target point
	 * @returns direction e.g.
	 *       0.5
	 *        ||
	 *  1/-1--  --0
	 *        ||
	 *      -0.5
	 */
	double getDirection(double p1_x, double p1_y, double p2_x, double p2_y);

	/**
	 * Returns the distance between 2 points.
	 * @param p1_x: x coordinate of first point
	 * @param p1_y: y coordinate of first point
	 * @param p2_x: x coordinate of second point
	 * @param p2_y: y coordinate of second point
	 * @return distance in meters
	 */
	double getDistance(double p1_x, double p1_y, double p2_x, double p2_y);

	/**
	 * Calculates the orientation difference between the given orientation and the given direction.
	 * If difference is positive the robot should turn counter-clockwise
	 * to head to the direction as far as possible, else negative -> clockwise.
	 * @param orientation: real orientation e.g.
	 * @param direction: desired direction e.g.
	 * @return orientation difference given as direction e.g.
	 *       0.5
	 *        ||
	 *  1/-1--  --0
	 *        ||
	 *      -0.5
	 */
	double getOrientationDiff(double orientation, double direction);

	/**
	 * Returns the angle of the given orientation.
	 * @param orientation: orientation e.g
	 *       0.5
	 *        ||
	 *  1/-1--  --0
	 *        ||
	 *      -0.5
	 *
	 * @returns an orientation angle e.g.
	 *      90
	 *      ||
	 * 180--  --0
 	 *      ||
	 *     270
	 */
	double getAngle(double orientation);

	/// information about the current role of the agent
	auto_smart_factory::RobotConfiguration robotConfig;

	/// Publisher for the motion actuator topic
	ros::Publisher* motionPub;

	/// start position of the current path
	geometry_msgs::Point startPosition;

	/// the current path to drive
	std::vector<geometry_msgs::Point> path;

	/// the index of the current waypoint
	int currentTargetIndex = -1;

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

	/// maximal speed to turn the robot on spot with
	double turningMaxSpeed = 1;

	/// minimal speed to turn the robot on spot with
	double turningMinSpeed = 0.02;

	/// maximal speed to drive straight with the robot
	double drivingMaxSpeed = 1;

	/// minimal speed to drive straight with the robot
	double drivingMinSpeed = 0.2;

	/// current turning speed of the robot - left > 0 > right
	double turnDirection = 0;

	/// Flag whether turning just started
	bool turningStart = true;

	/// Flag whether the robot turns left right now
	bool turnsLeft = true;

	/// Number of turns the robot did while current turning process
	int numberTurns = 0;

	/**
	 * Accuracy that determines the allowed distance to stop when in vicinity of the goal point while
	 * driving forward.
	 */
	double driveForwardDistanceAccuracy = 0.15;

	/**
	 * Accuracy that determines the allowed distance to stop when in vicinity of the goal point while
	 * driving backwards.
	 */
	double driveBackwardsDistanceAccuracy = 0.3;

	/**
	 * Accuracy that determines the allowed orientaion difference of the current robot
	 * orientation while driving & the direction of the goal position to not steer.
	 */
	double steeringAccuracy = 0.01;

	/**
	 * Accuracy that determines the allowed orientation difference
	 * to stop if the difference is small enough while turning on spot.
	 */
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
