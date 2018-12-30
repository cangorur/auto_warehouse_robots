#ifndef AGENT_CHARGINGMANAGEMENT_H_
#define AGENT_CHARGINGMANAGEMENT_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/RobotHeartbeat.h>

class Agent;
/**
 * The charging management component manages all charging stations, provides information about free
 * charging stations and offers a service to reserve charging stations.
 */
class ChargingManagement {
public:
	/**
	 * Default constructor.
	 * Sets up the initialize service.
	 */
	ChargingManagement(Agent* agent);

	virtual ~ChargingManagement();

private:

	Agent* agent;

	std::string agentID;

	float agentBatt;

	/// Subscriber to robot heartbeat topic
	ros::Subscriber robotHeartbeatSub;


	/**
	 * Receive robot heartbeats.
	 * @param hb Heartbeast message
	 */
	void receiveRobotHeartbeat(const auto_smart_factory::RobotHeartbeat& hb);

	/*
	 * Check battery and issue charging task accordingly
	 */
	void checkBattery(float);

};

#endif /* AGENT_CHARGINGMANAGEMENT_H_ */
