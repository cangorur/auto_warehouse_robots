#ifndef AUTO_SMART_FACTORY_SRC_WAREHOUSEMANAGEMENT_H_
#define AUTO_SMART_FACTORY_SRC_WAREHOUSEMANAGEMENT_H_

#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>

#include "auto_smart_factory/GetWarehouseConfig.h"
#include "auto_smart_factory/GetRobotConfigurations.h"
#include "auto_smart_factory/GetPackageConfigurations.h"
#include "auto_smart_factory/initRoadmapPlanner.h"
#include "auto_smart_factory/InitTaskPlanner.h"
#include "auto_smart_factory/InitAgent.h"
#include "auto_smart_factory/InitPackageGenerator.h"
#include "auto_smart_factory/InitStorageManagement.h"
#include "auto_smart_factory/InitChargingManagement.h"
#include "auto_smart_factory/triggerRoadmapGenerator.h"    // added 09.12.17 by Ansgar

#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/RobotConfiguration.h>
#include <auto_smart_factory/PackageConfiguration.h>
#include <auto_smart_factory/RobotHeartbeat.h>
#include <auto_smart_factory/TaskPlannerState.h>
#include <cstdlib>
#include <string>
#include <vector>

/**
 * This class initializes all other components and creates the abstract visualization of the warehouse.
 */
class WarehouseManagement{
public:
	WarehouseManagement();

	virtual ~WarehouseManagement();

	/**
	 * Start the warehouse and initialize all components.
	 */
	void start();

protected:
	/**
	 * Get all configurations from configuration server.
	 */
	void getConfigs();

	/**
	 * Get warehouse configuration from configuration server.
	 */
	bool getWarehouseConfiguration();

	/**
	 * Get robot configurations from configuration server.
	 */
	bool getRobotConfigurations();

	/**
	 * Get package configurations from configuration server.
	 */
	bool getPackageConfigurations();

	/**
	 * Initialize road map planner component
	 * This is not used in this version of the code. We have roadmap generator, path planner, traffic planner
 	 * all separated as different nodes.
	 */
	bool initRoadmapPlanner(auto_smart_factory::WarehouseConfiguration warehouse_configuration, std::vector<auto_smart_factory::RobotConfiguration> robot_configurations);

    /**
    * Initialize the roadmap generator.
    */
    bool initRoadmapGenerator(auto_smart_factory::WarehouseConfiguration warehouse_configuration);


	/**
	 * Initialize task planner component.
	 */
	bool initTaskPlanner(auto_smart_factory::WarehouseConfiguration warehouse_configuration, std::vector<auto_smart_factory::RobotConfiguration> robot_configurations, std::vector<auto_smart_factory::PackageConfiguration> package_configurations);

	/**
	 * Initialize agent/robot.
	 */
	bool initAgent(std::string agent_id, auto_smart_factory::WarehouseConfiguration warehouse_configuration, auto_smart_factory::RobotConfiguration robot_configuration);

	/**
	 * Initialize package generator component.
	 */
	bool initPackageGenerator(auto_smart_factory::WarehouseConfiguration warehouse_configuration, std::vector<auto_smart_factory::PackageConfiguration> package_configurations);

	/**
	 * Initialize storage management component.
	 */
	bool initStorageManagement(auto_smart_factory::WarehouseConfiguration warehouse_configuration, std::vector<auto_smart_factory::PackageConfiguration> package_configurations);

	/**
	 * Initialize charging management component.
	 */
	bool initChargingManagement(auto_smart_factory::WarehouseConfiguration warehouse_configuration);

	/**
	 * Initialize agent/robot.
	 */
	bool initWarehouseGateway(auto_smart_factory::WarehouseConfiguration warehouse_configuration);
	
	ros::NodeHandle n;

	/// The warehouse configuration
	auto_smart_factory::WarehouseConfiguration warehouseConfig;

	/// The robot configurations
	std::vector<auto_smart_factory::RobotConfiguration> robotConfigs;

	/// The package configurations
	std::vector<auto_smart_factory::PackageConfiguration> packageConfigs;

	/**
	 * Publishes the markers for abstract visualization.
	 * @param e
	 */
	void publishVisualization(const ros::TimerEvent& e);

	/**
	 * Creates the visualization marker for a tray.
	 */
	void createTrayMarker();

	/**
	 * Receive robot heartbeats.
	 * @param hb Heartbeat message
	 */
	void receiveHeartbeat(auto_smart_factory::RobotHeartbeat hb);

	/**
	 * Compote battery level to color mapping.
	 * @param batteryLevel Battery level
	 * @return Color
	 */
	static std_msgs::ColorRGBA batteryLevelToColor(double batteryLevel);

	/**
	 * Receive task planner state.
	 * @param msg task planner state
	 */
	void receiveTaskPlannerState(auto_smart_factory::TaskPlannerState msg);

	/// Publisher of the visualization markers
	ros::Publisher markerPub;

	/// Publisher of the occupancy map
	ros::Publisher occupancyMapPub;

	/// Robot heartbeat subscriber
	ros::Subscriber robotHeartbeatSub;

	/// Task planner state subscriber
	ros::Subscriber taskplannerStateSub;

	/// Visualization update timer
	ros::Timer vizPublicationTimer;

	/// Static warehouse markers
	visualization_msgs::MarkerArray trayMarkers;
};

#endif /* AUTO_SMART_FACTORY_SRC_WAREHOUSEMANAGEMENT_H_ */
