#ifndef AUTO_SMART_FACTORY_SRC_AGENT_H_
#define AUTO_SMART_FACTORY_SRC_AGENT_H_

#include "agent/MotionPlanner.h"
#include "agent/Gripper.h"
#include "agent/ObstacleDetection.h"
#include "agent/task_handling/TaskHandler.h"
#include "agent/ChargingManagement.h"

#include <random>
#include "ros/ros.h"
#include <string>
#include <deque>
#include <vector>
#include <exception>
#include <sstream>
#include <time.h>
#include "tf/transform_datatypes.h"
#include <stack>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "auto_smart_factory/RobotHeartbeat.h"
#include "auto_smart_factory/GripperState.h"
#include "auto_smart_factory/AdditionalTime.h"
#include "auto_smart_factory/InitAgent.h"
#include "auto_smart_factory/RegisterAgent.h"
#include "auto_smart_factory/StorePackage.h"
#include "auto_smart_factory/RetrievePackage.h"
#include "auto_smart_factory/AssignTask.h"
#include "auto_smart_factory/PerformTaskTest.h"
#include "auto_smart_factory/WarehouseConfiguration.h"
#include "auto_smart_factory/RobotConfiguration.h"
#include "auto_smart_factory/CollisionAction.h"
#include "auto_smart_factory/TaskAnnouncement.h"
#include "auto_smart_factory/TaskEvaluation.h"
#include "agent/path_planning/ReservationManager.h"
#include "agent/path_planning/Map.h"
#include "agent/path_planning/RobotHardwareProfile.h"

// forward declaration of TaskHandler class
class TaskHandler;

/* The agent component manages all robot related stuff and holds a motion planner, obstacle detection &
 * gripper instance. Furthermore it subscribes to robots sensor topics like pose, laser & battery sensor
 * and aggregates and analyses those data to be able to react to certain circumstances realized by
 * motion acutator and gripper. The agent implementation currently handles the charging request and
 * communications with the charging manager to assign for a charging task (a station as a destination) */
class Agent {
public:
	/* Constructor that sets up the initialize service and hands over the agents id.
	 * @param agent_id: id of this agent */
	Agent(std::string agent_id);

	virtual ~Agent();

	/* Returns ID of this agent */
	std::string getAgentID();

	/* Returns ID of this agent */
	int getAgentIdInt();

	/* Returns Battery of this agent */
	float getAgentBattery();

	/* Returns current position of this agent */
	geometry_msgs::Point getCurrentPosition();

	/* Returns current orientation of this agent */
	geometry_msgs::Quaternion getCurrentOrientation();
	
	OrientedPoint getCurrentOrientedPosition() const;

	/* Manages the current state of the agent and defines which actions to execute in order to
	 * keep this agent doing what it is supposed to. E.g. publish the heartbeat at every step.
	 * That is why it is called every tick (see AgentNode.cpp). */
	void update();

	/* Returns the tray with the given tray id.
	 * @param tray_id: id of the specified tray
	 * @return tray with the specified id */
	auto_smart_factory::Tray getTray(unsigned int tray_id);

	ros::Publisher* getVisualisationPublisher();

	std_msgs::ColorRGBA getAgentColor();
	
protected:

	bool init(auto_smart_factory::InitAgent::Request& req, auto_smart_factory::InitAgent::Response& res);

	/* Initializes this agent and sets up sensors & actuators.
	 * @param warehouse_configuration: information about the current warehouse map
	 * @param robot_configuration: information about the robot role this agent has */
	bool initialize(auto_smart_factory::WarehouseConfiguration warehouse_configuration, auto_smart_factory::RobotConfiguration robot_configuration);

	bool isInitializedCompletely();
	
	/* Calls a service to register the agent at the task planner, so this agent is able to receive
	 * requests to calculate and fullfill warehouse tasks.
	 * @return True if this agent has been registered successfully at the task planner. */
	bool registerAgent();

	/* Sets up the services to be able to receive task related requests by the task planner. */
	void setupTaskHandling();

	/* Returns whether it's time for next heartbeat to send.
	 * @return True if next heartbeat should be sended */
	bool isTimeForHeartbeat();

	/* Packs current agent state and some additional information & publishes it to the heartbeat topic.
	 * The information will directly be used by the path planner for the current position information of the agent.
	 * It will is also used by the task planner to decide on which agent is the most available (see the TaskPlanner.cpp)
	 * Therefore, a workload of the agent (OR an estimated time to finish the new tas) needs to be informed to the task planner
	 * @todo An ETA or a buffer indicating the current workload (needs to be associated in a quantitative way) needs to be
	 * incorporated to the hearbeat. */
	void sendHeartbeat();

	/* Resets the timer. */
	void updateTimer();

	/* Assign task service handler. If agent is idle it replaces the current plan with the plan
	 * corresponding to the given task, that has been calculated previously and should have
	 * been stored before in the tasks list. This is service called by the Task Planner, thru
	 * Request handler (see Request.cpp)
	 * @param req Request object
	 * @param res Request object
	 * @return True if the task has successfully been assigned */
	bool assignTask(auto_smart_factory::AssignTask::Request& req, auto_smart_factory::AssignTask::Response& res);

	/* Pose sensor callback handler. Calls the update function of the motion planner.
	 * @param msg: information about the position & orientation of the robot on the map */
	void poseCallback(const geometry_msgs::PoseStamped& msg);

	/* Laser sensor callback handler. Calls the update function of the obstacle detection.
	 * @param msg: sensed laser data */
	void laserCallback(const sensor_msgs::LaserScan& msg);

	/* Battery sensor callback handler. Saves battery level.
	 * @param msg: battery level [0.0 - 100.0] */
	void batteryCallback(const std_msgs::Float32& msg);
	
	/* Collision msg Callback handler. Disables the obstacle_detection and stops the motion_planner instances for the
	* time_to_halt specified in the msg. If there is not time_to_halt specified in the msg. That means that the agent has
	* to wait until a new path chunk is assigned to it to continue performing its task
	* @param msg: indicates to an agent for how long it has to halt
	* @todo this callback can be updated to take another strategies to avoid the a collision.
	**/
	void collisionAlertCallback(const auto_smart_factory::CollisionAction& msg);
	
	/*
	 * Publishes the visualisation message
	 */
	void publishVisualisation(const ros::TimerEvent& e);

	/*
	 * Callback for task Announcement messages computing the score and answering with score message
	 * @param taskAnnouncement: the taskAnnouncement message
	 */
	void announcementCallback(const auto_smart_factory::TaskAnnouncement& taskAnnouncement);
	
	// Reservation coordination
	void reservationBroadcastCallback(const auto_smart_factory::ReservationBroadcast& msg);

	// ROS Nodehandle
	ros::NodeHandle n;

	// ID of this agent
	std::string agentID;
	int agentIdInt;
	
	///////////////////////////////////////////////////////////
	Map* map;
	
	RobotHardwareProfile* hardwareProfile;
	
	// information about the current warehouse map
	auto_smart_factory::WarehouseConfiguration warehouseConfig;

	// information about the role of this agent
	auto_smart_factory::RobotConfiguration robotConfig;

	// Flag that shows whether this agent has been initialized already
	bool initialized = false;

	// Flag that shows whether this agent has already been registered at the task planner
	bool registered = false;

	// Publisher/Subscriber for ReservationCoordination
	ReservationManager* reservationManager;
	
	ros::Publisher reservationRequest_pub;
	ros::Subscriber reservationBroadcast_sub;

	// Server for initialization request
	ros::ServiceServer init_srv;

	// Server for store package request
	ros::ServiceServer store_package_srv;

	// Server for retrieve package request
	ros::ServiceServer retrieve_package_srv;

	// Server for assign task request
	ros::ServiceServer assign_task_srv;

	// Server for receive charging tasks from charging management
	ros::ServiceServer assign_charging_task_srv;

	// Subscriber for pose sensor topic
	ros::Subscriber pose_sub;

	// Subscriber for laser sensor topic
	ros::Subscriber hokuyo_sub;

	// Subscriber for battery sensor topic
	ros::Subscriber battery_sub;

	// Subscriber for CollisionAlert message which makes robot stop
	ros::Subscriber collision_alert_sub;

	// Subscriber for TaskHandler
	ros::Subscriber task_announce_sub;

	// Publisher for TaskHandler
	ros::Publisher taskrating_pub;

	// Ros visualisation
	ros::Publisher visualisationPublisher;
	ros::Timer vizPublicationTimer;

	// Publisher for motion actuator topic
	ros::Publisher motion_pub;

	// Publisher for gripper state topic
	ros::Publisher gripper_state_pub;

	// Publisher for heartbeat topic
	ros::Publisher heartbeat_pub;

	/// publisher for evaluation
	ros::Publisher taskEvaluation_pub;

	// pointer to instance of the Charging Management
	ChargingManagement* chargingManagement;

	// pointer to instance of the task planner
	TaskHandler* taskHandler;

	// pointer to instance of the motion planner
	MotionPlanner* motionPlanner;

	// pointer to instance of the gripper
	Gripper* gripper;

	// pointer to instance of obstacle detection
	ObstacleDetection* obstacleDetection;

	// current position of this agent
	geometry_msgs::Point position;

	// current orientation of this agent
	geometry_msgs::Quaternion orientation;
	
	// heartbeat related timestamp - time in seconds
	unsigned long lastHeartbeat = 0;

	// duration until the next heartbeat publishing - time in seconds
	unsigned long heartbeatPeriod = 0;

	// current battery level
	float batteryLevel = 100.0;

	// the color of the agent
	std_msgs::ColorRGBA agentColor;
};

#endif /* AUTO_SMART_FACTORY_SRC_AGENT_H_ */
