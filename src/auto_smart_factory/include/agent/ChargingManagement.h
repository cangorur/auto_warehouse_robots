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

	// Max energy level of the agent to participate in charging
	float upperThreshold = 90.00;

	// Energy level between upper and critical for non-linear score
	float lowerThreshold = 35.00;

	// Minimum energy level of the agent to participate in charging
	float criticalMinimum = 10.00;

	// Operating battery
	float operatingBatt = upperThreshold - criticalMinimum;

	//Current battery of the agent
	double agentBatt;

	//Estimated energy of the agent after task and charging
	float energyAfterTask;

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
};

#endif /* AGENT_CHARGINGMANAGEMENT_H_ */
