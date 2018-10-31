#ifndef AUTO_SMART_FACTORY_SRC_PLAN_H_
#define AUTO_SMART_FACTORY_SRC_PLAN_H_

#include "agent/MotionPlanner.h"
#include "agent/Gripper.h"
#include "agent/SubPlan.h"

#include "ros/ros.h"
#include <vector>
#include <string>
#include <sstream>

/**
 * The plan helps to predefine how a certain task could be fullfilled by an agent.
 * Possible tasks are the transport of packages, or to take care of the battery charge.
 * Each plan consists of a list of different subplans, but only one subplan may be active at a time.
 */
class Plan{

public:
	/**
	 * Default constructor.
	 * Initializes an empty plan and marks it as done.
	 */
	Plan();

	/**
	 * Constructor for a plan to drive to a certain goal position.
	 * @param agent_id: id of the agent this plan is created for
	 * @param motion_planner: instance of the motion planner to hand over instructions 
	 * 				related to the current subplan
	 * @param start_position: position where the robot should be located to start executing the plan
	 * @param path: path to drive from start position
	 * @param end_direction_point: point the robot should look at when the plan has been done
	 * @param length: estimated time duration in seconds to fullfill the plan
	 * @param drive_backwards: flag that tells whether to drive forwards or backwards. Provide true to drive backwards
	 */
	Plan(std::string agent_id, MotionPlanner &motion_planner, geometry_msgs::Point start_position, 
			std::vector<geometry_msgs::Point> path, geometry_msgs::Point end_direction_point, 
			double length,bool drive_backwards=false);

	/**
	 * Constructor for a plan to drive to a certain tray, grab a package there, transport it
	 * to another tray and release that package there.
	 * @param agent_id: id of the agent this plan is created for
	 * @param motion_planner: instance of the motion planner to hand over instructions 
	 * 		          related to the current subplan
	 * @param gripper: instance of the gripper to execute load / unload instructions
	 * @param storage_id: id of the tray where the package should be grabbed
	 * @param start_position: position where the robot should be located to start executing the plan
	 * @param path_to_load: path to drive from start position to the tray 
	 * 			where the package should be loaded
	 * @param load_direction_point: point the robot should look at when the package is grabbed
	 * @param path_to_unload: path to drive from the position the package has been loaded
	 * 			  to the tray where the package should be unloaded
	 * @param unload_direction_point: point the robot should look at when the package is released
	 * @param length: estimated time duration in seconds to fullfill the plan
	 */
	Plan(std::string agent_id, MotionPlanner &motion_planner, Gripper &gripper, 
			unsigned int storage_id, geometry_msgs::Point start_position, 
			std::vector<geometry_msgs::Point> path_to_load, 
			geometry_msgs::Point load_direction_point, 
			std::vector<geometry_msgs::Point> path_to_unload, 
			geometry_msgs::Point unload_direction_point, double length);

	/**
	 * Constructor for a plan to drive to a certain charging station, charge the battery there and
	 * drive back to the idle position after it.
	 * @param agent_id: id of the agent this plan is created for
	 * @param motion_planner: instance of the motion planner to hand over instructions 
	 * 		          related to the current subplan
	 * @param charging_station_id: id of the charging station where to charge
	 * @param start_position: position where the robot should be located to start executing the plan
	 * @param path_to_charging_station: path to drive from start position to the charging station 
	 * @param charging_direction_point: point the robot should look at to charge
	 * @param path_to_idle_position: path to drive from the charging station to the idle position
	 * @param end_direction_point: point the robot should look at when at the idle position
	 * @param length: estimated time duration in seconds to fullfill the plan
	 */
	Plan(std::string agent_id, MotionPlanner &motion_planner, unsigned int charging_station_id, 
			geometry_msgs::Point start_position, 
			std::vector<geometry_msgs::Point> path_to_charging_station, 
			geometry_msgs::Point charging_direction_point, 
			std::vector<geometry_msgs::Point> path_to_idle_position, 
			geometry_msgs::Point end_direction_point, double length);

	Plan(std::string agent_id, MotionPlanner &motion_planner, Gripper &gripper, bool load_action,
		geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> path,
		geometry_msgs::Point direction_point, geometry_msgs::Point drive_back_point, double length);

    Plan(std::string agent_id, MotionPlanner &motion_planner, unsigned int charging_station_id,
		geometry_msgs::Point start_position,
		std::vector<geometry_msgs::Point> path_to_charging_station,
		geometry_msgs::Point charging_direction_point, double length);

	virtual ~Plan();

	/**
	 * Modifies this plan such that it aims to load the battery immediately.
	 * @param agent_id: id of the agent this plan is created for
	 * @param charging_station_id: id of the charging station where to charge
	 * @param path_to_charging_station: path to drive from current position to the charging station 
	 * @param charging_direction_point: point the robot should look at to charge
	 * @param path_to_goal_position: path to drive from the charging station to the goal position
	 * @param goal_direction_point: point the robot should look at when at the goal position
	 * @param length: estimated time duration in seconds to fullfill the plan
	 * @todo dont overwrite current path but merge both paths (line 153)
	 */
	void insertCharging(std::string agent_id, unsigned int charging_station_id, 
			std::vector<geometry_msgs::Point> path_to_charging_station, 
			geometry_msgs::Point charging_direction_point, 
			std::vector<geometry_msgs::Point> path_to_goal_position, 
			geometry_msgs::Point goal_direction_point, double length);

     /**
	 * Modifies this plan such that it aims to load the battery immediately.
	 * @param agent_id: id of the agent this plan is created for
	 * @param charging_station_id: id of the charging station where to charge
	 * @param path_to_charging_station: path to drive from current position to the charging station
	 * @param charging_direction_point: point the robot should look at to charge
	 * @param path_to_goal_position: path to drive from the charging station to the goal position
	 * @param goal_direction_point: point the robot should look at when at the goal position
	 * @param length: estimated time duration in seconds to fullfill the plan
	 */
	void insertUn_LoadTask(std::string agent_id,
        bool isLoadPlan,
		geometry_msgs::Point position,
		geometry_msgs::Point direction_point,
		double length);

    /**
	 * Modifies this plan such that it aims to load the battery immediately.
	 * @param charging_station_id: id of the charging station where to charge
	 */
    void insertChargingSimple(unsigned int charging_station_id,
    double length);

	/**
	 * Executes this plan, handling & managing the subplans, checking whether 
	 * the active subplan has been fullfilled & instructs motion planner as well as gripper, 
	 * dependent on the current subplan.
	 * Should be called every tick of the agent.
	 * @param positon: current position of the agent
	 * @param battery_level: current battery level of the agent
	 * @todo Maybe the same workaround like with forward driving (line 228)
	 */
	void execute(geometry_msgs::Point position, float battery_level);

	/**
	 * Returns whether this plan has been fullfilled.
	 * @return True if plan is done
	 */
	bool isDone();

	/**
	 * Sets whether this plan has been fullfilled or not.
	 * @param done: whether plan is done or not
	 */
	void setDone(bool done);

	/**
	 * Returns the current active subplan.
	 * @return the current subplan
	 */
	SubPlan getCurrentSubPlan();

	/**
	 * Returns the estimated time in seconds to fullfill this plan.
	 * @return estimated time for this plan
	 */
	double getLength();

	/**
	 * Returns if the currently active subplan is a driving forward plan.
	 * Usefull because charging subplans are only going to be inserted while driving forward.
	 * @return True if current active subplan is of type drive
	 */
	bool isDrivingForward();

	/**
	 * Returns if the current subplans aim to charge the robot.
	 * @return True if the current subplan is related to charging stuff
	 */
	bool isCharging();

	/**
	 * Related to task handling.
	 * Returns the id of the tray where to grab a package.
	 * @return tray id
	 */
	unsigned int getStorageID();

protected:
	/**
	 * Euclidean distance between the 2 given points.
	 * @param p1: first point
	 * @param p2: second point
	 * @return distance between the points in meters
	 */
	double getDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);

	/// pointer to the motion planner instance
	MotionPlanner *motionPlanner;

	/// pointer to the gripper instance
	Gripper *gripper;

	/// ID of the agent this plan is created for
	std::string agentID = "none";

	/// Flag that shows if it's the first execution of this plan
	bool start = true;

	/// Flag that shows if this plan has already been fullfilled
	bool done = false;

	/// Flag that shows whether the current subplan is related to charging purposes
	bool isChargin = false;

	/// Start position of this plan
	geometry_msgs::Point startPosition;

	/// List of subplans this plan consists of
	std::vector<SubPlan> subplans;

	/// Index of the current subplan in the subplans list
	unsigned int currentIndex = 0;

	/// Estimated time duration in seconds considered for this plan 
	double length = 0;

	/// ID of the tray where a package should be loaded
	unsigned int storageID;

	/// Point the agent should look to when a package should be grabbed
	geometry_msgs::Point loadDirectionPoint;

	/// Point the agent should look to when a package should be released 
	geometry_msgs::Point unloadDirectionPoint;
};
#endif /* AUTO_SMART_FACTORY_SRC_PLAN_H_ */
