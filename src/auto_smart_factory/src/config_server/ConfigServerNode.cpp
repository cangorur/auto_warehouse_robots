#include <ros/ros.h>
#include <config_server/MapConfigServer.h>
#include <config_server/RobotConfigServer.h>
#include <config_server/PackageConfigServer.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, argv[1]);
	ros::NodeHandle nh;

	// start all configuration servers
	MapConfigServer mapServer;
	RobotConfigServer robotServer;
	PackageConfigServer packageServer;

	ros::spin();
}
