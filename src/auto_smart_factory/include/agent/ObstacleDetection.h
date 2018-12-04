#ifndef AUTO_SMART_FACTORY_SRC_OBSTACLEDETECTION_H_
#define AUTO_SMART_FACTORY_SRC_OBSTACLEDETECTION_H_

#include "agent/MotionPlanner.h"

#include "ros/ros.h"
#include <string>
#include <vector>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include "sensor_msgs/LaserScan.h"
#include <auto_smart_factory/RobotConfiguration.h>
#include <auto_smart_factory/WarehouseConfiguration.h>

/**
 * The obstacle detection component analyzes the laser data provided by the laser sensor,
 * compares the sensed data with the static obstacles on the map & sends local path requests to the 
 * roadmap planner if some dynamic obstacle has been sensed, so if it's necesarry an alternative
 * path could be chosen for the robot.
 */
class ObstacleDetection{

public:
	/**
	 * Constructor that hands over a motion planner instance and some configurations.
	 * Builds a grid matrix considering the given occupancy grid.
	 * @param agent_id: id of the agent this obstacle detection instance belongs to
	 * @param motion_planner: instance of the motion planner to hand over instructions 
	 * @param robot_config: information about the role of the agent this obstacle detection belongs to
	 * @param warehouse_config: information about the current warehouse map
	 */
	ObstacleDetection(std::string agent_id, 
			MotionPlanner &motion_planner, 
			auto_smart_factory::RobotConfiguration robot_config, 
			auto_smart_factory::WarehouseConfiguration warehouse_config);

	virtual ~ObstacleDetection();

	/**
	 * Hands over the current position, the current orientation of the robot and the current 
	 * laser data sensed by the laser sensor and decides whether to analyze the laser data.
	 * @param position: current positon of the agent
	 * @param orientation: current orientation of the agent
	 * @param msg: current laser data
	 */
	void update(geometry_msgs::Point& position, double orientation, 
			const sensor_msgs::LaserScan& msg);

	/**
	 * Enables / disables this obstacle detection instance.
	 * @param enable: whether to enable or disable
	 */
	void enable(bool enable);

	/**
	 * Returns whether this obstacle detection instance is enabled or not.
	 * @return enabled?
	 */
	bool isEnabled();

protected:

	/**
	 * Randomly calculates the next laser analyze range in the globally defined interval.
	 */
	void setNextAnalyzeRange();

	/**
	 * Analyzes whether the obstacles the laser sensed nearer to the robot then the given distance.
	 * @param distance: allowed distance
	 * @param ranges: list of the ranges each laser senses
	 * @return True if there are some obstacles in a smaller range then the given distance
	 */
	bool isRangeSmaller(double distance, std::vector<float> ranges);

	/**
	 * Maintains the whole analyzation process of the laser data, when there are obstacles detected
	 * @param positon: current position of the robot
	 * @param orientation: current orientation of the robot
	 * @param msg: currently sensed laser data
	 * @todo Come up with strategies when the obstacles are unknown to the agent (probably another agent)
	 * Todo so please go to the function dealWithObstacles.
	 */
	void analyzeLaserScan(geometry_msgs::Point& position, double orientation, 
			const sensor_msgs::LaserScan& msg);

	/**
	 * Translates the given laser ranges into points in the x-y coordinate plane considering the
	 * current pose of the robot.
	 * @param position: the current positon of the robot
	 * @param orientation: the curren orientation of the robot
	 * @param msg: the currently sensed laser data
	 * @return list of the points the lasers point to
	 */
	std::vector<geometry_msgs::Point> getPointList(geometry_msgs::Point position, double orientation,
		       	const sensor_msgs::LaserScan& msg);

	/**
	 * Compares the given points with the static obstacles of the current map given by the 
	 * occupancy grid.
	 * @param points: points the lasers point to
	 * @return list of detected dynamic obstacles associated 
	 * 	   with the border laser indexes that point to them
	 *  	   (right and left border index of each obstacle)
	 */
	std::vector<std::vector<int>> detectObstacle(std::vector<geometry_msgs::Point> &points);

	/**
	 * Your strategies to avoid such dynamic unknown obstacles.
	 * @param obstacles: list of obstacles given by the sensed obstacle border points
	 * @todo fill this function with your own strategies. Remember that if you implement a solution here
	 * then it is a decentralized local decision-making for the robot. If you call a service from another
	 * central agent to deal with, then it is a centralized decision-making. In any cases, a communication
	 * protocol (a hand-shake) between the agents (in case the obstacle is another agent) is strongly suggested here. 
	 */
	void dealWithObstacles(std::vector<geometry_msgs::Point> obstacles);

	/// id of the agent this obstacle detection instance belongs to
	std::string agentID; 

	/// pointer to the motion planner instance
	MotionPlanner *motionPlanner;

	/// information about the role of the agent this obstacle detection instance belongs to
	auto_smart_factory::RobotConfiguration robotConfig;

	/// information about the current warehouse map
	auto_smart_factory::WarehouseConfiguration warehouseConfig;

	/// the build up occupancy grid matrix
	std::vector<std::vector<float>> occupancyGrid;

	/// current analyze range
	double analyzeRange = 1.0;

	/// minimal analyze range
	double analyzeRangeMin = 1.5;

	/// maximal analyze range
	double analyzeRangeMax = 3.0;

	/// Flag whether this obstacle is enabled or not
	bool enabled = false;

	/// current position of the robot
	geometry_msgs::Point position;

	/// current orientaition of the robot
	double orientation;

	/// Flag whether an obstacle has been currently detected
	bool obstacleDetected = false;

	/// pi
	const double PI = 3.141592653589793238463;
};

#endif /* AUTO_SMART_FACTORY_SRC_OBSTACLEDETECTION_H_ */
