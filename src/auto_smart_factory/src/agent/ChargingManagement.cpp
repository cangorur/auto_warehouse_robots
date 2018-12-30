#include <agent/ChargingManagement.h>
#include "agent/Agent.h"


ChargingManagement::ChargingManagement(Agent* a) {

	ros::NodeHandle n;
	ros::NodeHandle pn("~");


	robotHeartbeatSub = n.subscribe("robot_heartbeats", 1000, &ChargingManagement::receiveRobotHeartbeat, this);
	agent = a;
	agentID = agent->getAgentID();
	agentBatt = agent->getAgentBattery();
	checkBattery(agentBatt);
//
//	ROS_INFO("[ChargingManagement] Started, agent ID: [%d], agent Batt: [%f] ", agentID, agentBatt);

}

ChargingManagement::~ChargingManagement() {
}


void ChargingManagement::receiveRobotHeartbeat(const auto_smart_factory::RobotHeartbeat& hb) {

//	ROS_INFO(
//			"[HeartBeat] New heartbeat received by agent [%s} , Battery is: [%f]", hb.id,  hb.battery_level);
	agentBatt = agent->getAgentBattery();
//	ROS_INFO("[HeartBeat] New heartbeat received, Battery is: [%f]", agentBatt);

	checkBattery(agentBatt);


}

//void ChargingManagement::getBatteryLevel() {
//
////	ROS_INFO(
////			"[HeartBeat] New heartbeat received, Battery is: [%f]", hb.battery_level);
//	float batteryLevel = hb.battery_level;
//	checkBattery(batteryLevel);
//}
//

void ChargingManagement::checkBattery(float batteryLevel){

	if(batteryLevel <= 10){

		//if batt <= 10, issue charging task immediately

		ROS_ERROR("[Charging Management] Battery Critical!! [%f]", batteryLevel);

	} else if(batteryLevel < 60 && batteryLevel > 10){

		//if 10 < batt <60 && if idle, decide to charge or not based on distance to the charging station

		ROS_INFO("[Charging Management] Battery discharging!! [%f]", batteryLevel);
	}


	// ID of the Charging station assigned to the agent
//	unsigned int chargingStationId;


	// if batt => 60, do not issue charging task
}
