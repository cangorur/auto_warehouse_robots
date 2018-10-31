/*
 * RobotConfigServer.cpp
 *
 *  Created on: 01.06.2017
 *      Author: jacob
 */

#include <config_server/RobotConfigServer.h>
#include <boost/property_tree/json_parser.hpp>

using namespace boost::property_tree;

RobotConfigServer::RobotConfigServer() {
	ros::NodeHandle nh("~");
	std::string robotConfigFileName;

	if(!nh.getParam("robot_config_file", robotConfigFileName)) {
		ROS_FATAL("No robot configuration file name given!");
		ros::shutdown();
		return;
	}

	readRobotConfigs(robotConfigFileName);

	configService = nh.advertiseService("get_robot_configurations", &RobotConfigServer::configCallback, this);
}

RobotConfigServer::~RobotConfigServer() {
}

bool RobotConfigServer::configCallback(auto_smart_factory::GetRobotConfigurations::Request& req, auto_smart_factory::GetRobotConfigurations::Response& res) {
	res.configs = robotConfigs;
	return true;
}

void RobotConfigServer::readRobotConfigs(std::string file) {
	ptree configTree;

	try {
		read_json(file, configTree);
	} catch(json_parser::json_parser_error &e) {
		ROS_FATAL("Cannot read robot configuration file %s. Message: %s", file.c_str(), e.what());
		return;
	}

	robotConfigs.clear();

	for(const auto& robotType : configTree) {
		auto_smart_factory::RobotConfiguration robotConfig;
		robotConfig.type_name = robotType.first;
		robotConfig.discharging_rate = robotType.second.get<float>("discharging_rate");
		robotConfig.charging_rate = robotType.second.get<float>("charging_rate");
		robotConfig.min_linear_vel = robotType.second.get<float>("min_linear_vel");
		robotConfig.max_linear_vel = robotType.second.get<float>("max_linear_vel");
		robotConfig.max_angular_vel = robotType.second.get<float>("max_angular_vel");
		robotConfig.radius = robotType.second.get<float>("radius");
		robotConfig.max_load = robotType.second.get<float>("max_load");

		robotConfigs.push_back(robotConfig);
	}
}
