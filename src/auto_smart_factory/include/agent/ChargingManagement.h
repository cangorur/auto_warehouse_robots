#ifndef AGENT_CHARGINGMANAGEMENT_H_
#define AGENT_CHARGINGMANAGEMENT_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/RobotHeartbeat.h>
#include "auto_smart_factory/Tray.h"

class ChargingStation {
public:
	//Corresponding Charging tray
	auto_smart_factory::Tray Tray;

	//Is occupied or not
	bool occupancy;

	//Assigned Robot
	auto_smart_factory::Robot robot;

	//Charging Rate //TODO??
	float rate;

};

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
	ChargingManagement(Agent* agent, auto_smart_factory::WarehouseConfiguration warehouse_configuration);

	virtual ~ChargingManagement();

	/* Returns true if the agent can perform the task of given energy
	 * @param energy: expected energy of the task
	 */
	bool isEnergyAvailable(double energy);

	/*
	 * Score multiplier for current set of tasks +  charging
	 * @param Energy consumption of all the tasks to be done
	 * @returns Score multiplier LOWER IS BETTER
	 */
	float getScoreMultiplier(float cumulatedEnergyConsumption);

	/*
	 * Get All Charging Stations
	 */
	void getAllChargingStations();

private:
	Agent* agent;

	//Agent ID from where the CM is called
	std::string agentID;

	// information about the current warehouse map
	auto_smart_factory::WarehouseConfiguration warehouseConfig;

	// Max energy level of the agent to participate in charging
	float upperThreshold = 90.00;

	// Energy level between upper and critical for non-linear score
	float lowerThreshold = 35.00;

	// Minimum energy level of the agent to participate in charging
	float criticalMinimum = 10.00;

	//Vector of all the Charging Trays
	std::vector <auto_smart_factory::Tray> charging_trays;

	//Vector of all Charging Stations'
	std::vector <ChargingStation> charging_stations;

	//Current battery of the agent
	double agentBatt;

	//Estimated energy of the agent after task and charging
	float energyAfterTask;
};



#endif /* AGENT_CHARGINGMANAGEMENT_H_ */
