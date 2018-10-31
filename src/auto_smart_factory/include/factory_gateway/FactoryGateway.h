/*
 * TaskPlanner.cpp
 *
 *  Created on: 25.08.2017
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */
#ifndef AUTO_SMART_FACTORY_SRC_FACTORY_GATEWAY_FACTORYGATEWAY_H_
#define AUTO_SMART_FACTORY_SRC_FACTORY_GATEWAY_FACTORYGATEWAY_H_

#include <ros/ros.h>
#include <memory>
#include <thread>
#include "std_msgs/Float32.h"
//#include <auto_smart_factory/Tray.h>
//#include <auto_smart_factory/StorageUpdate.h>
#include <auto_smart_factory/GetStorageState.h>
#include <auto_smart_factory/GetFreeChargingStations.h>
#include <auto_smart_factory/TrayState.h>
#include <auto_smart_factory/NewPackageInput.h>
#include <auto_smart_factory/NewPackageGenerator.h>
//#include <auto_smart_factory/GET_STORAGE_MANAGEMENT> // TODO: this is to be found ! Check out the service defined and 
#include <boost/property_tree/json_parser.hpp>
/*#include "simple_web_socket/server_ws.hpp"
#include "simple_web_socket/client_ws.hpp"

typedef SimpleWeb::SocketServer<SimpleWeb::WS> WsServer;
typedef SimpleWeb::SocketClient<SimpleWeb::WS> WsClient;
*/
//*/

/**
 * The task planner component manages all incoming requests, checks for resources
 * and starts new tasks.
 */
class FactoryGateway {
public:
	FactoryGateway();
	virtual ~FactoryGateway();
	
	/**
	 * Initialize web socket server
	 * @return True if initialization was successful
	 */
	void SetWebSocketServer();
	
	void webSocketClient(std::string message);
	

private:
	/**
	 * Initialize service handler.
	 * @param req Request object
	 * @param res Response object
	 * @return True if initialization was successful
	 */
	void initialize();

	/**
	 * Get current storage state from the storage management. Takes a json ptree and returns it back with storage info
	 * @return Storage state
	 */
	boost::property_tree::ptree GetStateOfStorages(boost::property_tree::ptree message_pt);
	
	/*
	 * Get current input container states from the storage management. Takes a json ptree and returns it back with container info
	 * @return Input Containers state
	 */
	boost::property_tree::ptree GetStateOfInputContainers(boost::property_tree::ptree message_pt);
	
	/*
	 * Get current delivery container states from the storage management. Takes a json ptree and returns it back with container info
	 */
	boost::property_tree::ptree GetStateOfDeliveryContainers(boost::property_tree::ptree message_pt);
	
	/**
	 * Get state of charging stations, occupied or not. Takes a json ptree and returns it back with station info
	 * @return Storage state
	 */
	boost::property_tree::ptree GetStateOfChargingStations(boost::property_tree::ptree message_pt);
	
	/**
	 * Get state of delivery robots. Takes a json ptree and returns it back with robot info
	 * @return Delivery robot states
	 */
	boost::property_tree::ptree GetStateOfDeliveryRobots(boost::property_tree::ptree message_pt);
	
	/**
	 * Set state of conveyor belts. Takes a json ptree and returns it back with toggle positions info
	 * @return Conveyor belt states
	 */
	boost::property_tree::ptree ToggleConveyorBelts(boost::property_tree::ptree message_pt);
	
	/**
	 * Receive robot battery levels.
	 * @param std_msgs::Float32 data
	 */
	void receiveRobot1Battery(const std_msgs::Float32& msg);
	void receiveRobot2Battery(const std_msgs::Float32& msg);
	void receiveRobot3Battery(const std_msgs::Float32& msg);
	void receiveRobot4Battery(const std_msgs::Float32& msg);
	void receiveRobot5Battery(const std_msgs::Float32& msg);
	void receiveRobot6Battery(const std_msgs::Float32& msg);
	void receiveRobot7Battery(const std_msgs::Float32& msg);
	void receiveRobot8Battery(const std_msgs::Float32& msg);
	
	
private:

	/// subscriber to the storage update topic
	ros::Subscriber storageUpdateSub;
	
	/// Service for getting tray (stroge) states
	ros::ServiceClient storageStates;
	
	/// Service for free charging stations
	ros::ServiceClient freeChargingStations;
	
	/// Service for toggling conveyor belts
	ros::ServiceClient toggleConveyor1;
	ros::ServiceClient toggleConveyor2;
	
	/// Requesting for new task for an input package (storing)
	ros::ServiceClient generateNewPackageClient;
	
	/// Subscriber for delivery robots' battery levels
	ros::Subscriber robot1_battery;
	ros::Subscriber robot2_battery;
	ros::Subscriber robot3_battery;
	ros::Subscriber robot4_battery;
	ros::Subscriber robot5_battery;
	ros::Subscriber robot6_battery;
	ros::Subscriber robot7_battery;
	ros::Subscriber robot8_battery;
	
	/// Websocket server object
	
	double robotBatteries_arr[8] = {100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0}; //for 8 different robots
	std::string conveyor1_state = "off";
	std::string conveyor2_state = "off";
	std::string conveyor1_speed = "0.0";
	std::string conveyor2_speed = "0.0";
	// WsServer server;
	// WsClient client

};

#endif /* AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASKPLANNER_H_ */
