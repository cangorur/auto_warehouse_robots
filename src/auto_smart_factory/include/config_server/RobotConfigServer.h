/*
 * RobotConfigServer.h
 *
 *  Created on: 01.06.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_ROBOTCONFIGSERVER_H_
#define AUTO_SMART_FACTORY_SRC_ROBOTCONFIGSERVER_H_

#include <ros/ros.h>
#include <auto_smart_factory/RobotConfiguration.h>
#include <auto_smart_factory/GetRobotConfigurations.h>

/**
 * This class reads the robot configuration file and provides a service to deliver the robot
 * configurations to other components.
 */
class RobotConfigServer {
public:
	RobotConfigServer();
	virtual ~RobotConfigServer();

protected:
	/**
	 * Robot configurations service callback function.
	 * @param req Request object
	 * @param res Response object
	 * @return Success (always true)
	 */
	bool configCallback(auto_smart_factory::GetRobotConfigurations::Request& req, auto_smart_factory::GetRobotConfigurations::Response& res);

	/**
	 * Reads the JSON formatted robot configs from file.
	 * @param file path to configuration file
	 */
	void readRobotConfigs(std::string file);

	/**
	 * Robot configurations
	 */
	std::vector<auto_smart_factory::RobotConfiguration> robotConfigs;

	/**
	 * Robot configurations retrieval server
	 */
	ros::ServiceServer configService;
};

#endif /* AUTO_SMART_FACTORY_SRC_ROBOTCONFIGSERVER_H_ */
