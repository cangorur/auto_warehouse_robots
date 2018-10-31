#ifndef AUTO_SMART_FACTORY_SRC_PATH_H
#define AUTO_SMART_FACTORY_SRC_PATH_H

#include "ros/ros.h"
#include <string>
#include <exception>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "auto_smart_factory/RequestNewPath.h"
#include <auto_smart_factory/RobotConfiguration.h>

enum GOAL
{
    LOAD,
    UNLOAD,
    CHARGE,
    IDLE
};

class Path{
public:
	/**
	 * Constructor.
	 * @param agent_id: id of this agent
	 * @param robot_config: information about the robot role this agent has
	 * @param task_id: the id of the specified task
	 * @param start_position: start position of the path
	 * @param end_position: the last point to which the robot will drive
	 * @param approach_point: a point in front of a tray, the robot drives to
	 *       this point guided by path planner. Then it drives straight
	 *       from this point to the end_position
	 * @param direction_point: this the actual position of a tray, which
	 *       is indeed covered by the tray itself
	 * @param drive_back_point: after loading/unloading the robot drives
	 *       backwards to this point from where it can go somewhere else
	 * @param idle: indicates if the agent is carrying a package or not
	 */
	Path(std::string agent_id, int task_id,  GOAL path_goal, geometry_msgs::Point start_position,
        geometry_msgs::Point end_position, geometry_msgs::Point approach_point,
        geometry_msgs::Point direction_point, geometry_msgs::Point drive_back_point, bool idle);
	Path();
	virtual ~Path();

	/**
	 *
	 */
	bool isDone();
	void setDone(bool done);

	/**
	*
	**/
	bool isFirstChunk();
	void setFirstChunk(bool firstChunk);

    /**
    *
    **/
    int getTaskId();

    /**
    *
    **/
    bool isIdle();

    /**
    *
    **/
    GOAL getPathGoal();

    /**
    *
    **/
    bool isLastChunk(geometry_msgs::Point position);

    geometry_msgs::Point getDirectionPoint();
    geometry_msgs::Point getDriveBackPoint();
    geometry_msgs::Point getApproachPoint();
    geometry_msgs::Point getStartPosition();
    geometry_msgs::Point getEndPosition();




protected:
    std::string agentID;

    int taskID;

	bool idle;

	bool done;

    /// if the path is newly started, i.e. random points
    /// have to be requested from the path planner
	bool firstChunk;

    int chunkIndex;

    /// load, unload, charge, idle
	GOAL pathGoal;

    /// start point of the path
    geometry_msgs::Point startPosition;

    /// the last point to which the robot will drive
    geometry_msgs::Point endPosition;

    /// actual tray position
	geometry_msgs::Point directionPoint;

	/// drive to this point before driving to endPosition
	/// in order to approach the storages in the right angle
	geometry_msgs::Point approachPoint;

    /// drive backwards to this point after (un)loading
	geometry_msgs::Point driveBackPoint;
};


#endif /* AUTO_SMART_FACTORY_SRC_PATH_H */