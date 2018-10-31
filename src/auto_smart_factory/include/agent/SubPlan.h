#ifndef AUTO_SMART_FACTORY_SRC_SUBPLAN_H_
#define AUTO_SMART_FACTORY_SRC_SUBPLAN_H_

#include "ros/ros.h"
#include <vector>
#include "geometry_msgs/Point.h"

/// types of possible subplans
enum SubPlanType {drive, backwards, load, unload, charge, unknown, Default = unknown};

/**
 * The subplan is a part of a plan. It helps to focus the agent on one certain task at a time.
 * At most one subplan can be active for each agent at a time.
 */
class SubPlan{

public:
	/**
	 * Constructor for a driving forward or backward subplan.
	 * @param path: the path that should be driven
	 * @param end_direction_point: the point the agent should look to after the path has been driven
	 * @param drive_backwards: whether path should be driven forward or backwards
	 */
	SubPlan(std::vector<geometry_msgs::Point> path, geometry_msgs::Point end_direction_point, 
			bool drive_backwards);

	/**
	 * Constructor for a load or unload package subplan.
	 * @param do_load: whether the package should be loaded or unloaded
	 * @param position: the location on the map where the agent should be located
	 * @param direction_point: the point the agent should look to
	 */
	SubPlan(bool do_load, geometry_msgs::Point position, geometry_msgs::Point direction_point);

	/**
	 * Constructor for a charging subplan.
	 * @param charging_station_id: the id of the charging station where to charge
	 */
	SubPlan(unsigned int charging_station_id);

	virtual ~SubPlan();

	/**
	 * Returns the type of the subplan.
	 * @return type of this subplan
	 */
	SubPlanType getType();

	/**
	 * Returns if this subplan has already been fullfilled.
	 * @return True if plan has already been done
	 */
	bool isDone();

	/**
	 * Sets this subplan to be done or not.
	 * @param done: whether subplan should be marked as done or not
	 */
	void setDone(bool done);

	/**
	 * Related to driving forward or backward subplans.
	 * Returns the total path of the current subplan.
	 * @return list of points
	 */
	std::vector<geometry_msgs::Point> getPath();

	/**
	 * Related to driving forward or backward subplans or load / unload subplans.
	 * Returns the endposition / grip position of this subplan.
	 * @return location where the agent should be located after this subplan is done
	 */
	geometry_msgs::Point getEndPosition();

	/**
	 * Related to driving forward or backward subplans or load / unload subplans.
	 * Returns the end direction point of this subplan.	 
	 * @return the location the agent should look to when this subplan has been done.
	 */
	geometry_msgs::Point getEndDirectionPoint();

	/**
	 * Related to charging subplans.
	 * Returns the ID of the station to charge at.
	 * @return charging station ID
	 */
	unsigned int getChargingStationId();

protected:
	/// Type of this subplan
	SubPlanType type;

	/// Flap if this subplan has been done already
	bool done = false;

	/// List of points to drive along
	std::vector<geometry_msgs::Point> path;

	/// Point the agent should look to when the plan is done (or before loading unloading)
	geometry_msgs::Point endDirectionPoint;

	/// Point the agent should be located at to trigger the gripper service
	geometry_msgs::Point gripPosition;

	/// ID of the charging station the agent should be charged at
	unsigned int chargingStationID;
};
#endif /* AUTO_SMART_FACTORY_SRC_SUBPLAN_H_ */
