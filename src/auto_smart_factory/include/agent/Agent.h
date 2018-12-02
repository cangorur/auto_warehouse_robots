#ifndef AUTO_SMART_FACTORY_SRC_AGENT_H_
#define AUTO_SMART_FACTORY_SRC_AGENT_H_

#include "agent/MotionPlanner.h"
#include "agent/Gripper.h"
#include "agent/ObstacleDetection.h"

#include <random>
#include "ros/ros.h"
#include <string>
#include <deque>
#include <vector>
#include <exception>
#include <sstream>
#include <time.h>
#include <tf/transform_datatypes.h>
#include <stack>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "auto_smart_factory/RobotHeartbeat.h"
#include "auto_smart_factory/GripperState.h"
#include "auto_smart_factory/AdditionalTime.h"
#include "auto_smart_factory/InitAgent.h"
#include "auto_smart_factory/RegisterAgent.h"
#include "auto_smart_factory/shortestPath.h"
#include "auto_smart_factory/bestGoal.h"
#include "auto_smart_factory/StorePackage.h"
#include "auto_smart_factory/RetrievePackage.h"
#include "auto_smart_factory/AssignTask.h"
#include "auto_smart_factory/PerformTaskTest.h"
#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/RobotConfiguration.h>
#include "auto_smart_factory/RegisterAgentCharging.h"
#include "auto_smart_factory/AssignChargingTask.h"
#include <auto_smart_factory/CollisionAction.h>

/// defines the task id type
typedef uint32_t TaskId;

/**
 * The agent component manages all robot related stuff and holds a motion planner, obstacle detection &
 * gripper instance. Furthermore it subscribes to robots sensor topics like pose, laser & battery sensor
 * and aggregates and analyses those data to be able to react to certain circumstances realized by
 * motion acutator and gripper. The agent implementation currently handles the charging request and
 * communications with the charging manager to assign for a charging task (a station as a destination)
 */
class Agent{
public:
	/**
	 * Constructor that sets up the initialize service and hands over the agents id.
	 * @param agent_id: id of this agent
	 */
	Agent(std::string agent_id);
	virtual ~Agent();

	/**
	 * Returns ID of this agent
	 */
	std::string getAgentID();

	/**
     * Returns current position of this agent
     */
	geometry_msgs::Point getCurrentPosition();

	/**
     * Returns current orientation of this agent
     */
	geometry_msgs::Quaternion getCurrentOrientation();

	/**
	 * Manages the current state of the agent and defines which actions to execute in order to
	 * keep this agent doing what it is supposed to. E.g. publish the heartbeat at every step.
	 * That is why it is called every tick (see AgentNode.cpp).
	 */
	void update();

protected:

	bool init(auto_smart_factory::InitAgent::Request  &req, auto_smart_factory::InitAgent::Response &res);

	/**
	 * Initializes this agent and sets up sensors & actuators.
	 * @param warehouse_configuration: information about the current warehouse map
	 * @param robot_configuration: information about the robot role this agent has
	 */
	bool initialize(auto_smart_factory::WarehouseConfiguration warehouse_configuration,
			auto_smart_factory::RobotConfiguration robot_configuration);

	/**
	 * Extracts the idle position of this agent from the warehouse config.
	 * @return True if idle postion extracted successfully.
	 */
	bool setupIdlePosition();

	/**
	 * Calls a service to register the agent at the task planner, so this agent is able to receive
	 * requests to calculate and fullfill warehouse tasks.
	 * @return True if this agent has been registered successfully at the task planner.
	 */
	bool registerAgent();

	/**
	 * Sets up the services to be able to receive task related requests by the task planner.
	 */
	void setupTaskHandling();

	/**
	 * Sets if this agent is idle or not and if the state has been changed it sends a heartbeat.
	 * @param idle: whether to set to idle state
	 */
	void setState(bool idle);

	/**
	 * Returns whether it's time for next heartbeat to send.
	 * @return True if next heartbeat should be sended
	 */
	bool isTimeForHeartbeat();

	/**
	 * Packs current agent state and some additional information & publishes it to the heartbeat topic.
	 * The information will directly be used by the path planner for the current position information of the agent.
	 * It will is also used by the task planner to decide on which agent is the most available (see the TaskPlanner.cpp)
	 * Therefore, a workload of the agent (OR an estimated time to finish the new tas) needs to be informed to the task planner
	 * @todo An ETA or a buffer indicating the current workload (needs to be associated in a quantitative way) needs to be
	 * incorporated to the hearbeat.
	 */
	void sendHeartbeat();

	/**
	 * Resets the timer.
	 */
	void updateTimer();

	/**
	 * Handles the idle state which leads to driving back to the robots idle position.
	 */
	void handleIdleState();

	/**
	 * Store package service handler.
	 * @param req Request object
	 * @param res Response object
	 * @return True if store package task calculation has been succesful
	 */
	bool storePackage(auto_smart_factory::StorePackage::Request &req,
			auto_smart_factory::StorePackage::Response &res);

	/**
	 * Retrieve package service handler.
	 * @param req Request object
	 * @param res Response object
	 * @return True if retrieve package task calculation has been succesful
	 */
	bool retrievePackage(auto_smart_factory::RetrievePackage::Request  &req,
			auto_smart_factory::RetrievePackage::Response &res);

	/**
	 * Assign task service handler. If agent is idle it replaces the current plan with the plan
	 * corresponding to the given task, that has been calculated previously and should have
	 * been stored before in the tasks list. This is service called by the Task Planner, thru
	 * Request handler (see Request.cpp)
	 * @param req Request object
	 * @param res Request object
	 * @return True if the task has successfully been assigned
	 */
	bool assignTask(auto_smart_factory::AssignTask::Request  &req,
			auto_smart_factory::AssignTask::Response &res);

	/**
	 * Returns the tray with the given tray id.
	 * @param tray_id: id of the specified tray
	 * @return tray with the specified id
	 */
	auto_smart_factory::Tray getTray(unsigned int tray_id);

	/**
	 * Pose sensor callback handler. Calls the update function of the motion planner.
	 * @param msg: information about the position & orientation of the robot on the map
	 */
	void poseCallback(const geometry_msgs::PoseStamped& msg);

	/**
	 * Laser sensor callback handler. Calls the update function of the obstacle detection.
	 * @param msg: sensed laser data
	 */
	void laserCallback(const sensor_msgs::LaserScan& msg);

	/**
	 * Battery sensor callback handler. Saves battery level.
	 * @param msg: battery level [0.0 - 100.0]
	 */
	void batteryCallback(const std_msgs::Float32& msg);

    /**
    * Collision msg Callback handler. Disables the obstacle_detection and stops the motion_planner instances for the
    * time_to_halt specified in the msg. If there is not time_to_halt specified in the msg. That means that the agent has
    * to wait until a new path chunk is assigned to it to continue performing its task
	* @param msg: indicates to an agent for how long it has to halt
	* @todo this callback can be updated to take another strategies to avoid the a collision.
    **/
    void collisionAlertCallback(const auto_smart_factory::CollisionAction& msg);

	/**
    * Calculates the time to run a distance given distance and velocity
	* @param double: distance value
	* @param double: velocity value
	* @return double value representing the calculated time
    **/
    double calculateTimeFromDistanceAndVelocity(double distance, double velocity);

    /**
    * Generate random float number between min and max (used for local collision handling)
	* @param min: minimum float value
	* @param max: maximum float value
	* @return random generated float number
    **/
   float randomFloat (float min, float max);

	/// ROS Nodehandle
	ros::NodeHandle n;

	/// ID of this agent
	std::string agentID;

	/// information about the current warehouse map
	auto_smart_factory::WarehouseConfiguration warehouseConfig;

	/// information about the role of this agent
	auto_smart_factory::RobotConfiguration robotConfig;

	/// the idle position of this agent on the current map
	geometry_msgs::Point idlePosition;

	/// the point the agent should look to if it is on idle position
	geometry_msgs::Point idleOrientationPoint;

	/// Flag that shows whether this agent has been initialized already
	bool initialized = false;

	/// Flag that shows whether this agent has already been registered at the task planner
	bool registered = false;

	/// Flag that shows whether this agent has already been registered at the task planner
	bool registeredCharging = false;

	/// Flag indicated whether to insert charging or using other approaches
	bool insertCharging = true;

	/// Flag that shows if the robot is currently in idle state or not
	bool isIdle = true;

	/// Server for initialization request
	ros::ServiceServer init_srv;

	/// Server for store package request
	ros::ServiceServer store_package_srv;

	/// Server for retrieve package request
	ros::ServiceServer retrieve_package_srv;

	/// Server for assign task request
	ros::ServiceServer assign_task_srv;

	/// Server for receive charging tasks from charging management
    ros::ServiceServer assign_charging_task_srv;

	/// Subscriber for pose sensor topic
	ros::Subscriber pose_sub;

	/// Subscriber for laser sensor topic
	ros::Subscriber hokuyo_sub;

	/// Subscriber for battery sensor topic
	ros::Subscriber battery_sub;

	/// Subscriber for CollisionAlert message which makes robot stop
	ros::Subscriber collision_alert_sub;

	/// Publisher for motion actuator topic
	ros::Publisher motion_pub;

	/// Publisher for gripper state topic
	ros::Publisher gripper_state_pub;

	/// Publisher for additional time topic - sent if some task is delayed because of charging
	ros::Publisher additional_time_pub;

	/// Publisher for heartbeat topic
	ros::Publisher heartbeat_pub;

	/// pointer to instance of the motion planner
	MotionPlanner *motionPlanner;

	/// pointer to instance of the gripper
	Gripper *gripper;

	/// pointer to instance of obstacle detection
	ObstacleDetection *obstacleDetection;

	/// current position of this agent
	geometry_msgs::Point position;

	/// current orientation of this agent
	geometry_msgs::Quaternion orientation;

	/// heartbeat related timestamp - time in seconds
	unsigned long lastTimestamp = 0;

	/// duration until the next heartbeat publishing - time in seconds
	unsigned long breakDuration = 0.5;

	/// current battery level
	float batteryLevel = 100.0;

	/// when battery level is lower than threshold it's necessary to take care of charging
	float batteryThreshold = 20.0;

	/// Flag that shows if the agent is waiting for a charging response
	bool waitingForChargingRequest = false;

	// indicates whether the agent has ever moved
	bool hasDriven;

	/// Indicates whether to generate a charging plan or not
	bool generateChargingPlan = false;

	/// ID of the Charging station assigned to the agent
	unsigned int chargingStationId;

	/// value of PI
	const double PI = 3.141592653589793238463;

	/// counter to indicate how long an agent stuck in one place (used for local collision handling for static/dynamic obstacles)
	int stuck_Counter = 0;

	/// charging task id
	int chargeTaskID = 99999;

	/// interval since last time of check local collision
	int poseSampleInterval = 0;

	/// current position of this agent (sampled in local collisition check)
	geometry_msgs::Point sample_position;

	/// current orientation of this agent (sampled in local collision check)
	geometry_msgs::Quaternion sample_orientation;

    /// Stores the time when the current task was assigned to the agent
	float initialTimeOfCurrentTask = -1.0;

};
#endif /* AUTO_SMART_FACTORY_SRC_AGENT_H_ */
