#include <agent/ChargingManagement.h>

ChargingManagement::ChargingManagement() {

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	ROS_INFO("[ChargingManagement] Started");

	robotHeartbeatSub = n.subscribe("robot_heartbeats", 10, &ChargingManagement::receiveRobotHeartbeat, this);


}

ChargingManagement::~ChargingManagement() {
}


void ChargingManagement::receiveRobotHeartbeat(const auto_smart_factory::RobotHeartbeat& hb) {

	ROS_INFO(
			"[HeartBeat] New heartbeat received, Battery is: %f", hb.battery_level);
}
