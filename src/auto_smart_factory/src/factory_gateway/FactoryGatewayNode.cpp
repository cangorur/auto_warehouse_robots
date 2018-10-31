/*
 * WareHouseGatewayNode.cpp
 *
 *  Created on: 25.08.2017
 *      Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <factory_gateway/FactoryGateway.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "factory_gateway");
	ros::NodeHandle nh;
	
	FactoryGateway factoryGateway;
  	ROS_INFO("Factory Gateway is ready!");
  	factoryGateway.SetWebSocketServer();
  	// Ros::Spin() carried out into WareHouseGateway class, SetWebSocket function under its on_message() thread. So that whenever a message arrives ros spins and gets uptodate rostopic callbacks
}