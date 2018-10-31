#ifndef AUTO_SMART_FACTORY_SRC_CHARGINGMANAGEMENT_H_
#define AUTO_SMART_FACTORY_SRC_CHARGINGMANAGEMENT_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include "auto_smart_factory/InitChargingManagement.h"
#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/RobotHeartbeat.h>
#include "auto_smart_factory/RegisterAgentCharging.h"
#include "auto_smart_factory/AssignChargingTask.h"
#include "auto_smart_factory/GetFreeChargingStations.h"

/**
 * The charging management component manages all charging stations, provides information about free
 * charging stations and offers a service to reserve charging stations.
 */
class ChargingManagement{
public:
	/**
	 * Default constructor.
	 * Sets up the initialize service.
	 */
	ChargingManagement();
	virtual ~ChargingManagement();

		/**
	 * Manages the current state of the charging stations.
	 */
	void update();

protected:
	/**
	 * Initialize service handler.
	 * @param req Request object
	 * @param res Response object
	 * @return True if initialization was successful
	 */
	bool init(auto_smart_factory::InitChargingManagement::Request &req, 
			auto_smart_factory::InitChargingManagement::Response &res);
	/**
	 * Initializes charging stations & services.
	 * @param warehouse_configuration information about the current warehouse map
	 * @return True if initialization was succesful
	 */
	bool initialize(auto_smart_factory::WarehouseConfiguration warehouse_configuration);

	/**
	 * Initializes charging stations.
	 */
	void initializeChargingStations();

	/**
	 * Receive robot heartbeats. Each agent node publishes to the same topic
	 * @todo set warning battery level according to robots config (line 120)
	 * It should be tested if all of them are successfully caught
	 * @param hb Heartbeast message
	 */
	void receiveRobotHeartbeat(const auto_smart_factory::RobotHeartbeat &hb);

	/**
	 * Called by robots. Registers a robot.
	 * @param req Request object
	 * @param res Response object
	 * @return Always true
	 */
	bool registerAgent(auto_smart_factory::RegisterAgentChargingRequest &req, auto_smart_factory::RegisterAgentChargingResponse &res);

	/**
	 * Assign charging task/ leave charging station request to robot.
	 *
	 * @param robotId Id of the robot
	 * @return True if assigning was successful
	 */
	bool assignRobot(std::string robotId,int tray_id, bool end) const;

	/**
	* Calculate best match charging station for one robot
	* @param robotId of the robot
	* @return index of the charging station in all free charging stations if successfull else return -1
	*/
    int findBestStation(std::string robotId);

	/**
	 * Get ETA for target robot to arrive charging station. This is used to select the best station (closest)
	 * @param robotId
	 * @param charging_position the coordinate of assigned charging station
	 * @return ETA value
	 * @todo eta server is to be implemented by the students. Please adjust the necessary
	 * parts mentioned in the function in the source (ChargingManagement.cpp line 151) to successfully call a ROS
	 * service to be implemented under ETA server.
	 */

	int getRobotETA(geometry_msgs::Point robotPose, geometry_msgs::Point charging_position);
	
	/**
	 * GetFreeChargingStations service handler.
	 * @param req Request object
	 * @param res Response object
	 * @return True if service has been called succesfully.
	 */
	bool getFreeChargingStations(auto_smart_factory::GetFreeChargingStations::Request &req, 
			auto_smart_factory::GetFreeChargingStations::Response &res);

	/// ROS Nodehandle
	ros::NodeHandle n;

	/// Flag wheter charging management has already been initialized
	bool initialized = false;

	/// Subscriber to the robot_heartbeats
	ros::Subscriber robotHeartbeatSub;

	/// Server for initialization
	ros::ServiceServer initServer; 

	/// Server for agent/robot registration
	ros::ServiceServer registerAgentChargingServer;

	/// Server for getting list of free charging stations
	ros::ServiceServer getFreeChargingStationsServer;

	/// All information about the current map
	auto_smart_factory::WarehouseConfiguration warehouseConfig;

	/// List of reserved charging stations
	std::vector<auto_smart_factory::Tray> reservedChargingStations;

	/// List of free charging stations
	std::vector<auto_smart_factory::Tray> freeChargingStations;

	/// List of occupied charging stations
	std::map<std::string, auto_smart_factory::Tray> occupiedChargingStations;

	/// all registered robots: mapping their id to their configuration and their status (status are "untracked", "unassigned", "assigned", "requested", "charged")
	std::map< std::string, std::pair<auto_smart_factory::RobotConfiguration, std::string> > registeredRobots;

	/// all robots that needs charging or are charging: mapping their id to their assigned charging station id
	std::map <std::string, int> trackedRobots;

	/// Tentative: record robots position to decide best match
	std::map <std::string, geometry_msgs::Point> robotsPosition;

	/// Define charged battery level
	const float CHARGED = 100.0;

	/// Define critical battery level
	const float CRITICAL = 50.0;
};

#endif /* AUTO_SMART_FACTORY_SRC_CHARGINGMANAGEMENT_H_ */
