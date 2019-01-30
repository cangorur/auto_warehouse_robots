#ifndef AGENT_CHARGINGMANAGEMENT_H_
#define AGENT_CHARGINGMANAGEMENT_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/RobotHeartbeat.h>
#include "auto_smart_factory/Tray.h"
#include "agent/path_planning/Map.h"

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
	ChargingManagement(Agent* agent, auto_smart_factory::WarehouseConfiguration warehouse_configuration, Map* map);

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
	double getScoreMultiplier(float cumulatedEnergyConsumption);

	/*
	 * Get All Charging Stations
	 */
	void getAllChargingStations();

	/*
	 * Find all possible paths to the nearest charging stations and then return the shortest one
	 */
	Path getPathToNearestChargingStation(OrientedPoint start, double startingTime);

	/*
	 * Returns 100-Current Battery
	 */

	double getDischargedBattery();



private:
	Agent* agent;

	//Agent ID from where the CM is called
	std::string agentID;

	//Map of the system
	Map* map;

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

	//Current battery of the agent
	double agentBatt;

	//Estimated energy of the agent after task and charging
	float energyAfterTask;
};



#endif /* AGENT_CHARGINGMANAGEMENT_H_ */
