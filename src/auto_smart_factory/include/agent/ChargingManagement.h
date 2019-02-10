#ifndef AGENT_CHARGINGMANAGEMENT_H_
#define AGENT_CHARGINGMANAGEMENT_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/RobotConfiguration.h>
#include <auto_smart_factory/RobotHeartbeat.h>
#include "auto_smart_factory/Tray.h"
#include "agent/path_planning/Map.h"
#include "std_msgs/Float32.h"


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
	ChargingManagement(Agent* agent, auto_smart_factory::WarehouseConfiguration warehouse_configuration, auto_smart_factory::RobotConfiguration robot_configuration, Map* map);

	virtual ~ChargingManagement() = default;

	/*
	 * Score multiplier for current set of tasks + charging
	 * @param Energy consumption of all the tasks to be done
	 * @returns Score multiplier LOWER IS BETTER
	 */
	double getScoreMultiplierForBatteryLevel(double batteryLevel);

	/*
	 * Get All Charging Stations
	 */
	void getAllChargingStations();

	/*
	 * Find all possible paths to the nearest charging stations and then return the shortest one
	 * @param start: The point where the robo will be when it starts to drive to the nearest charging tray
	 * @param startingTime: the time at which the robot will start to drive to the nearest charging tray
	 * @returns a pair of the path and the id of the selected tray
	 */
	std::pair<Path, uint32_t> getPathToNearestChargingStation(OrientedPoint start, double startingTime);

	// Returns if charging should be done
	bool isChargingAppropriate();

	// Returns if the robot is sufficiently charged
	bool isCharged();

	// Returns the current battery of the agent
	double getCurrentBattery();

	// Checks if the battery consumption is possible
	bool isConsumptionPossible(double consumption);

	// Checks if the battery consumption is possible given the theoretical battery level
	bool isConsumptionPossible(double agentBatteryLevel, double consumption);

	/*
	 * Gets charging time
	 * @param: Battery consumption till robot reaches charging station
	 */
	double getChargingTime(double consumptionTillCS);

	/*
	 * Get battery consumption by the robot
	 * @param: Time robot spent idling
	 * @param: Distance traveled by the robot
	 */

private:
	Agent* agent;

	//Agent ID from where the CM is called
	std::string agentID;

	//Map of the system
	Map* map;

	// information about the current warehouse map
	auto_smart_factory::WarehouseConfiguration warehouseConfig;

	// information about the role of this agent
	auto_smart_factory::RobotConfiguration robotConfig;

	float dischargingRate;
	float chargingRate;
	float motorDrainingRate;

	float chargingAppropriateLevel = 96.f;
	
	// Above this value no score penalty is applied
	float upperThreshold = 50.00;

	// greater => quadratic function, smaller => linear function
	float lowerThreshold = 35.00;

	// Below minimum => no new tasks
	float criticalMinimum = 10.00;
	
	float estimatedBatteryConsumptionToNearestChargingStation = 10.f;

	//Vector of all the Charging Trays
	std::vector <auto_smart_factory::Tray> charging_trays;
};



#endif /* AGENT_CHARGINGMANAGEMENT_H_ */
