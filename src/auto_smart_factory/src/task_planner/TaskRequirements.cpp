/*
 * TaskRequirements.cpp
 *
 *  Created on: 11.07.2017
 *      Author: jacob
 */

#include "task_planner/TaskRequirements.h"

#include "ros/ros.h"
#include "auto_smart_factory/GetTrayState.h"

using namespace auto_smart_factory;

TaskRequirements::TaskRequirements(
		auto_smart_factory::PackageConfiguration pkgConfig)
		:
		pkgConfig(pkgConfig) {

}

TaskRequirements::~TaskRequirements() {
}

bool TaskRequirements::isLegalRobot(
		const auto_smart_factory::RobotConfiguration& robotConfig) const {
	return pkgConfig.weight <= robotConfig.max_load;
}

const auto_smart_factory::PackageConfiguration& TaskRequirements::getPackageConfig() const {
	return pkgConfig;
}

auto_smart_factory::TrayState TaskRequirements::getTrayState(
		unsigned int trayId) {
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<GetTrayState>(
			"/storage_management/get_tray_state");
	GetTrayState srv;
	srv.request.trayId = trayId;

	if(!client.call(srv)) {
		ROS_ERROR("Service call to get storage state failed!");
	}

	return srv.response.state;
}
